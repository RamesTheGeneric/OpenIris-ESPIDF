# Fix: Offload RS Decode to Dedicated Task

## Root Cause
The backtrace confirms: `tlsf_realloc` → `std::string` → `scan_start` → `sta_input`
The `new (std::nothrow)` at `wifiManager.cpp:392` and the RS decode loop run inside `sniffer_cb()` on the WiFi task, corrupting the WiFi driver's internal ring buffer heap. `taskYIELD()` doesn't help — we're still on the WiFi task's heap.

## Fix: Binary Semaphore + Decode Task

### Architecture
```
sniffer_cb (WiFi task)     →  decode_task (dedicated task)
─────────────────────────    ─────────────────────────────
- Parse headers             - Wait on binary semaphore
- Store chunk data          - RS decode (heap-safe)
- Track chunk map           - Find JPEG end marker
- Signal semaphore          - Call g_jpegFrameCallback
- NO heap allocation        - Clean up buffer
```

### File: wifiManager.cpp

#### 1. Add binary semaphore + decode task handle (after line 36):
```cpp
static BinarySemaphoreHandle_t rs_decode_semaphore;
static TaskHandle_t rs_decode_task = nullptr;
```

#### 2. Remove the entire decode block from `sniffer_cb()` (lines 389-436):
Replace with:
```cpp
    // Signal decode task when frame is ready (only once per frame)
    if (!frame_decoded && all_rs_blocks_decodable()) {
        frame_decoded = 1;
        xSemaphoreGiveFromISR(rs_decode_semaphore, pdFALSE);
    }
```

#### 3. Remove remaining `printf` from sniffer (line 433):
```cpp
        delete[] decoded;  // remove this line too since decode is offloaded
```

#### 4. Add decode task function (before `sniffer_cb`):
```cpp
static void rs_decode_task_fn(void* arg)
{
    while (true) {
        if (xSemaphoreTake(rs_decode_semaphore, portMAX_DELAY) != pdTRUE) continue;

        // Decode all RS blocks
        uint8_t* decoded = new (std::nothrow) uint8_t[FEC_MAX_RS_BLOCKS * FEC_RS_DATA_CHUNKS * MAX_PAYLOAD_SIZE];
        if (!decoded) continue;
        memset(decoded, 0, FEC_MAX_RS_BLOCKS * FEC_RS_DATA_CHUNKS * MAX_PAYLOAD_SIZE);

        size_t totalDecodedLen = 0;
        bool decodeOk = true;

        for (uint8_t b = 0; b < rs_block_count; b++) {
            size_t blockLen = 0;
            if (!rs_decode_block(b, decoded + totalDecodedLen, &blockLen)) {
                decodeOk = false;
                break;
            }
            totalDecodedLen += blockLen;
        }

        if (decodeOk) {
            // Find JPEG end marker
            size_t total_length = 0;
            for (size_t i = totalDecodedLen - 1; i >= 1; i--) {
                if (decoded[i-1] == 0xFF && decoded[i] == 0xD9) {
                    total_length = i + 1;
                    break;
                }
            }
            if (total_length == 0) total_length = totalDecodedLen;

            if (g_jpegFrameCallback) {
                g_jpegFrameCallback(decoded, total_length);
            }
        }

        delete[] decoded;
    }
}
```

#### 5. Initialize semaphore + create task in `Begin()` RX_MODE section (after line 468):
```cpp
rs_decode_semaphore = xSemaphoreCreateBinary();
xTaskCreatePinnedToCore(rs_decode_task_fn, "rs_decode", 4096, nullptr, 5, &rs_decode_task, 1);
```

#### 6. Remove `#include <new>` — decode is now off the WiFi task
Actually keep it — the decode task still needs `new (std::nothrow)`.

### File: main/openiris_main.cpp
No changes needed — callback already set before `Begin()`.

## Why This Works
- `sniffer_cb` is now purely heap-free: only `memcpy` to static buffers + `xSemaphoreGiveFromISR`
- RS decode runs on a dedicated task with its own heap context
- The `RS::ReedSolomon` constructor and `new[]` no longer corrupt the WiFi driver's ring buffer