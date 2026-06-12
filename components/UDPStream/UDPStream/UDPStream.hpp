#pragma once
#ifndef UDPSTREAM_HPP
#define UDPSTREAM_HPP

#include <cstdint>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "fec.h"

#define UDP_MAGIC           0x4952
#define UDP_RS_DATA_CHUNKS   4
#define UDP_RS_PARITY_CHUNKS 4
#define UDP_RS_TOTAL_CHUNKS  8
#define UDP_CHUNK_SIZE      350
#define UDP_MAX_RS_BLOCKS   16

struct __attribute__((packed)) udp_frame_hdr_t {
    uint16_t magic;
    uint8_t  frame_id;
    uint8_t  rs_block_id;
    uint8_t  chunk_id;
    uint16_t total_chunks;
    uint8_t  chunk_type;
    uint16_t chunk_len;
};

class UDPStream {
public:
    UDPStream();
    ~UDPStream();

    bool start(uint32_t client_ip, uint16_t client_port);
    void stop();
    bool isActive() const;

private:
    static void streamTaskFn(void* arg);
    bool sendFrame(int sock, const camera_fb_t* fb);

    TaskHandle_t _task = nullptr;
    volatile bool _active = false;
    uint32_t _client_ip = 0;
    uint16_t _client_port = 0;
    uint8_t _frame_id = 0;
    struct sockaddr_in _dest_addr;

    static constexpr const char* TAG = "[UDP_STREAM]";
};

#endif
