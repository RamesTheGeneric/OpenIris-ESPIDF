# Blank Video Feed Debug Plan

## Problem
ESP32 appears as a UVC camera device on the host computer, but the video feed is blank.
- "Submitted JPEG frame" logs appear (RS decode works, callback fires)
- "FB_CB" and "Failed to capture picture" logs appear when host has camera app open
- No "Camera Start" or "Configured UVC Device" logs seen by user

## Root Cause Analysis

### Confirmed Correct
- JPEG encoding/decoding chain is theoretically correct (verified by analysis)
- RS(8,4) FEC sends all 8 data chunks per block (TX pads with zeros)
- RX fast path copies all 8 chunks when all present
- End marker search correctly finds JPEG EOI
- `provide_jpeg_frame` correctly allocates and copies data
- Semaphore signaling is correct

### Key Findings
1. `camera_start_cb` is NEVER called — the UVC stream is never activated
2. This means the host-UVC protocol handshake isn't completing
3. The UVC device enumerates correctly (appears in device list)
4. But the host app can't activate the streaming interface

### Possible Causes
1. **UVC descriptor issue** — The USB descriptors may not match what the host expects
2. **TinyUSB/UVC driver issue** — The `tud_video_commit_cb` may not be triggered
3. **Host app compatibility** — The camera app may not support this UVC device
4. **Configuration issue** — `uvc_device_config` or `uvc_device_init` may be failing silently

## Debug Steps

### Step 1: Verify UVC Setup Logs
Check full serial boot output for:
- "Setting up UVC Stream"
- "Configured UVC Device"
- "Initialized UVC Device"
- Any "UVC Device failed" error messages

### Step 2: Add JPEG Data Integrity Logging
In `provide_jpeg_frame()`, add:
```cpp
ESP_LOGI(UVC_STREAM_TAG, "JPEG SOI: 0x%02X 0x%02X, EOI: 0x%02X 0x%02X",
         new_buf[0], new_buf[1],
         new_buf[jpeg_len - 2], new_buf[jpeg_len - 1]);
```
This verifies the decoded JPEG starts with `0xFFD8` (SOI marker) and ends with `0xFFD9` (EOI marker).

### Step 3: Add UVC Frame Acquisition Logging
In `camera_fb_get_cb()`, add logging:
```cpp
ESP_LOGI(UVC_STREAM_TAG, "fb_get_cb: sem=%s, buf=%p, len=%zu",
         "taken", (void*)jpeg_s_fb.buf, jpeg_s_fb.len);
```
This tracks when frames are acquired by the UVC stack and what buffer state is read.

### Step 4: Check Host Camera App
- Try multiple camera apps (Photo Booth, OBS, VLC, etc.)
- Check if the host app reports any errors
- Verify the device is listed as a valid camera in system settings

### Step 5: Check UVC Driver Initialization
Add error logging to `uvc_device_config` and `uvc_device_init` return values.

## Implementation Notes
- The `frame_ready_sem` timeout in `camera_fb_get_cb` is 100ms — frames arriving between UVC polls may be missed
- The UVC frame rate is 90fps (interval ~11ms) but frames arrive at ~10fps
- "Failed to capture picture" is expected for ~89% of UVC polls (frame rate mismatch)
- The critical issue is that `camera_start_cb` is never called, meaning the UVC stream is never activated

## Files to Modify
1. `components/UVCStream/UVCStream/UVCStream.cpp` — Add JPEG integrity logging and UVC frame acquisition logging
2. `components/UVCStream/UVCStream/UVCStream.cpp` — Verify `uvc_device_config` and `uvc_device_init` return values
