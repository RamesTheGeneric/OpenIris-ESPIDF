#pragma once
#ifndef TXSTREAM_HPP
#define TXSTREAM_HPP

#include "esp_log.h"
#include "esp_camera.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "rs.hpp"
#include <stdio.h>
#include <cstring>

#ifndef MAX_PAYLOAD_SIZE
#define MAX_PAYLOAD_SIZE 1400
#endif

// 802.11 Frame Control for data frames
#ifndef FRAME_CONTROL_DATA
#define FRAME_CONTROL_DATA 0x08
#endif

// RS(8,4) FEC parameters
// At each byte position, 8 data bytes are RS-encoded into 4 parity bytes.
// Up to 4 chunks per RS block can be lost and still recovered.
#define FEC_RS_DATA_CHUNKS   8
#define FEC_RS_PARITY_CHUNKS 4
#define FEC_RS_TOTAL_CHUNKS  12

// Maximum RS blocks per frame (4 RS blocks = ~45KB max JPEG)
#define FEC_MAX_RS_BLOCKS    4

static const uint8_t vendor_oui[3] = {0xAC,0xDE,0x47}; //Right = {0xAC,0xDE,0x48}  ||  Left == {0xAC,0xDE,0x47}

class TXStream {
public:
    // matches TXStream::TXstream in your .cpp
    explicit TXStream(int wifi_channel);

    // kicks off the continuous camera->WiFi loop
    void startStream();

    // called internally for each JPEG chunk
    static void send_jpeg_frame(const uint8_t *jpeg, size_t len);

    // FCS calculation for data frames
    static uint32_t calculate_fcs(const uint8_t *data, size_t len);

private:
    int _wifi_channel;

    // camera & JPEG buffering state
    camera_fb_t* fb        = nullptr;
    esp_err_t       response  = ESP_OK;
    const uint8_t*  _jpg_buf  = nullptr;
    size_t          _jpg_buf_len = 0;
    
    // 802.11 Data Frame header structure
    typedef struct __attribute__((packed)) {
        uint8_t  frame_control[2];
        uint8_t  duration[2];
        uint8_t  addr1[6];      // Destination
        uint8_t  addr2[6];      // Source
        uint8_t  addr3[6];      // BSSID
        uint8_t  seq_ctrl[2];
    } wifi_ieee80211_data_hdr_t;

    // Custom protocol header embedded in data payload (11 bytes total)
    typedef struct __attribute__((packed)) {
        uint8_t  oui[3];        // Vendor OUI
        uint8_t  frame_id;      // Frame identifier
        uint8_t  rs_block_id;   // Which RS block this chunk belongs to
        uint8_t  chunk_id;      // Chunk index within the RS block (0-11)
        uint8_t  total_chunks;  // Total chunks in this RS block (always 12)
        uint8_t  chunk_type;    // 0 = data chunk, 1 = parity chunk
        uint16_t chunk_len;     // Length of this chunk
    } custom_data_hdr_t;

    static constexpr const char* TAG = "TX_SERVER";
};

#endif // TXSTREAM_HPP