# Fix Plan: UVC No Output + Heap Crash

## Root Causes

### 1. UVC No Output — Callback Registered Too Late
In `main/openiris_main.cpp:231-237`:
```cpp
wifiManager.Begin();                  // line 231 — promiscuous callback fires immediately
// mdnsManager.start();               // line 232
// restAPI->begin();                  // line 233
wifiManager.setJpegFrameCallback(...);  // line 235 — callback set AFTER Begin()
```
Decoded frames at `wifiManager.cpp:444` check `if (g_jpegFrameCallback)` which is still `nullptr`, so frames are silently dropped.

### 2. Heap Corruption Crash
Backtrace: `tlsf_realloc` → `std::string` → `sniffer_cb` context.
The `RS::ReedSolomon<8,4>` decode loop runs inside `sniffer_cb()` on the WiFi task. `printf` calls trigger `std::string` allocations that corrupt the TLSF heap shared with the WiFi driver's ring buffer.

## Fixes

### Fix 1: Set callback before Begin()
**File:** `main/openiris_main.cpp` (lines 231-238)

Move `setJpegFrameCallback()` before `wifiManager.Begin()`:
```cpp
#ifdef CONFIG_RX_MODE
wifiManager.setJpegFrameCallback([&](uint8_t* frameBuffer, uint16_t length) {
    uvcStream.provide_jpeg_frame(frameBuffer, length);
});
ESP_LOGI("[MAIN]", "Set WiFi Manger jpeg callback");
#endif
wifiManager.Begin();
```

### Fix 2: Remove printf from WiFi callback context
**File:** `components/wifiManager/wifiManager/wifiManager.cpp`

Remove all `printf` calls from `sniffer_cb()` and `rs_decode_block()`. The WiFi promiscuous callback must be lightweight and heap-free.

Remove these lines:
- `sniffer_cb`: `printf("Invalid chunk length...")`, `printf("RS block ID out of range...")`, `printf("Frame buffer overflow...")`
- `rs_decode_block`: `printf("RS block %d...")`, `printf("RS decode failed...")`
- Decode completion: `printf("Complete JPEG frame...")`, `printf("Frame %d decode failed...")`

Keep only the "incomplete frame" log in `sniffer_cb` since it fires infrequently.

### Fix 3: Move RS decode off WiFi task (optional but recommended)
**File:** `components/wifiManager/wifiManager/wifiManager.cpp`

The `taskYIELD()` helps but doesn't prevent heap corruption since we're still on the WiFi task. Better approach: signal decode completion via a binary semaphore or queue, let a dedicated task do the heavy RS decode work.

However, Fix 1 + Fix 2 should be sufficient to resolve both issues.

## File Changes

| File | Change |
|------|--------|
| `main/openiris_main.cpp` | Move `setJpegFrameCallback()` before `Begin()` |
| `wifiManager.cpp` | Remove `printf` calls from `sniffer_cb()` and `rs_decode_block()` |
