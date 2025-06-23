#pragma once
#ifndef TXSTREAM_HPP
#define TXSTREAM_HPP

#include "esp_log.h"
#include "esp_camera.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_err.h"
#include <stdio.h>
#include <cstring>

// payload-size limit per 802.11 action frame
#ifndef MAX_PAYLOAD_SIZE
#define MAX_PAYLOAD_SIZE 200
#endif

// 802.11 Frame Control for vendor‐specific action
#ifndef FRAME_CONTROL
#define FRAME_CONTROL 0xD0
#endif

static const uint8_t vendor_oui[3] = {0xAC,0xDE,0x48}; 

class TXStream {
public:
    // matches TXStream::TXstream in your .cpp
    explicit TXStream(int wifi_channel);

    // kicks off the continuous camera->WiFi loop
    void startStream();

    // called internally for each JPEG chunk
    static void send_jpeg_frame(const uint8_t *jpeg, size_t len);

private:
    int _wifi_channel;

    // camera & JPEG buffering state
    camera_fb_t* fb        = nullptr;
    esp_err_t       response  = ESP_OK;
    const uint8_t*  _jpg_buf  = nullptr;
    size_t          _jpg_buf_len = 0;
    typedef struct __attribute__((packed)) {
    uint8_t  frame_control[2];
    uint8_t  duration[2];
    uint8_t  addr1[6];
    uint8_t  addr2[6];
    uint8_t  addr3[6];
    uint8_t  seq_ctrl[2];
    } wifi_ieee80211_hdr_t;

    static constexpr const char* TAG = "TX_SERVER";
};

#endif // TXSTREAM_HPP
