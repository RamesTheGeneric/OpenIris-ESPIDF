#include "UDPStream.hpp"
#include <StreamServer.hpp>
#include <lwip/def.h>
#include <errno.h>
#include "esp_wifi.h"

static fec_t* s_fec = nullptr;

UDPStream::UDPStream() {}

UDPStream::~UDPStream()
{
    stop();
}

bool UDPStream::start(uint32_t client_ip, uint16_t client_port)
{
    if (_active) {
        ESP_LOGW(TAG, "UDP stream already active");
        return false;
    }

    _client_ip = client_ip;
    _client_port = client_port;
    _frame_id = 0;

    _dest_addr.sin_family = AF_INET;
    _dest_addr.sin_port = htons(client_port);
    _dest_addr.sin_addr.s_addr = htonl(client_ip);
    memset(&_dest_addr.sin_zero, 0, sizeof(_dest_addr.sin_zero));

    _active = true;

    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "WiFi power save disabled");

    BaseType_t result = xTaskCreate(
        streamTaskFn, "udp_stream", 4096, this,
        5, &_task);

    if (result != pdPASS) {
        _active = false;
        esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
        ESP_LOGE(TAG, "Failed to create UDP stream task");
        return false;
    }

    StreamServer::setUdpActive(true);

    ESP_LOGI(TAG, "UDP stream started to %u.%u.%u.%u:%d",
             (client_ip >> 24) & 0xFF, (client_ip >> 16) & 0xFF,
             (client_ip >> 8) & 0xFF, client_ip & 0xFF, client_port);
    return true;
}

void UDPStream::stop()
{
    if (!_active) return;
    _active = false;

    if (_task) {
        xTaskNotifyGive(_task);
        for (int i = 0; i < 30; i++) {
            if (_task == nullptr) break;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (_task != nullptr) {
            vTaskDelete(_task);
            _task = nullptr;
        }
    }
    esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    StreamServer::setUdpActive(false);
    ESP_LOGI(TAG, "UDP stream stopped \u2014 WiFi power save restored");
}

bool UDPStream::isActive() const
{
    return _active;
}

static void rs_encode_block(
    const uint8_t* dataBuf,
    uint8_t* parityBuf)
{
    const gf* src[UDP_RS_DATA_CHUNKS];
    gf* dst[UDP_RS_PARITY_CHUNKS];
    unsigned block_nums[UDP_RS_PARITY_CHUNKS];

    for (uint8_t i = 0; i < UDP_RS_DATA_CHUNKS; i++)
        src[i] = dataBuf + i * UDP_CHUNK_SIZE;

    for (uint8_t i = 0; i < UDP_RS_PARITY_CHUNKS; i++) {
        dst[i] = parityBuf + i * UDP_CHUNK_SIZE;
        block_nums[i] = UDP_RS_DATA_CHUNKS + i;
    }

    fec_encode(s_fec, src, dst, block_nums, UDP_RS_PARITY_CHUNKS, UDP_CHUNK_SIZE);
}

static bool send_udp_chunk(int sock,
                            const struct sockaddr_in* dest_addr,
                            uint8_t frame_id, uint8_t rs_block_id,
                            uint8_t chunk_id, uint8_t chunk_type,
                            uint16_t total_chunks,
                            const uint8_t* data, uint16_t data_len)
{
    uint8_t packet[sizeof(udp_frame_hdr_t) + UDP_CHUNK_SIZE];
    auto* hdr = (udp_frame_hdr_t*)packet;

    hdr->magic = htons(UDP_MAGIC);
    hdr->frame_id = frame_id;
    hdr->rs_block_id = rs_block_id;
    hdr->chunk_id = chunk_id;
    hdr->total_chunks = htons(total_chunks);
    hdr->chunk_type = chunk_type;
    hdr->chunk_len = htons(data_len);

    memcpy(packet + sizeof(udp_frame_hdr_t), data, data_len);
    if (data_len < UDP_CHUNK_SIZE) {
        memset(packet + sizeof(udp_frame_hdr_t) + data_len, 0,
               UDP_CHUNK_SIZE - data_len);
    }

    uint16_t packet_len = sizeof(udp_frame_hdr_t) + UDP_CHUNK_SIZE;
    int sent = sendto(sock, packet, packet_len, MSG_DONTWAIT,
                      (const struct sockaddr*)dest_addr, sizeof(*dest_addr));
    return sent == packet_len;
}

bool UDPStream::sendFrame(int sock, const camera_fb_t* fb)
{
    if (!s_fec) {
        init_fec();
        s_fec = fec_new(UDP_RS_DATA_CHUNKS, UDP_RS_TOTAL_CHUNKS);
        if (!s_fec) {
            ESP_LOGE(TAG, "Failed to initialize zfec encoder");
            return false;
        }
    }

    size_t len = fb->len;
    const uint8_t* jpeg = fb->buf;

    if (!jpeg || len == 0) return false;

    uint8_t numDataChunks = (len + UDP_CHUNK_SIZE - 1) / UDP_CHUNK_SIZE;
    if (numDataChunks == 0) numDataChunks = 1;

    uint8_t numRsBlocks = (numDataChunks + UDP_RS_DATA_CHUNKS - 1) / UDP_RS_DATA_CHUNKS;
    if (numRsBlocks > UDP_MAX_RS_BLOCKS) numRsBlocks = UDP_MAX_RS_BLOCKS;

    uint16_t totalChunks = numRsBlocks * UDP_RS_TOTAL_CHUNKS;

    uint8_t* dataBuf = (uint8_t*)calloc(UDP_RS_DATA_CHUNKS * UDP_CHUNK_SIZE, 1);
    uint8_t* parityBuf = (uint8_t*)calloc(UDP_RS_PARITY_CHUNKS * UDP_CHUNK_SIZE, 1);

    if (!dataBuf || !parityBuf) {
        ESP_LOGE(TAG, "Failed to allocate RS buffers");
        free(dataBuf);
        free(parityBuf);
        return false;
    }

    const uint8_t* jpegPtr = jpeg;
    size_t jpegRemaining = len;
    uint16_t dropped = 0;

    for (uint8_t block = 0; block < numRsBlocks; block++) {
        memset(dataBuf, 0, UDP_RS_DATA_CHUNKS * UDP_CHUNK_SIZE);

        size_t blockDataSize = UDP_RS_DATA_CHUNKS * UDP_CHUNK_SIZE;
        size_t copySize = (jpegRemaining < blockDataSize) ? jpegRemaining : blockDataSize;
        memcpy(dataBuf, jpegPtr, copySize);

        jpegPtr += copySize;
        jpegRemaining -= copySize;

        memset(parityBuf, 0, UDP_RS_PARITY_CHUNKS * UDP_CHUNK_SIZE);

        rs_encode_block(dataBuf, parityBuf);

        uint8_t data_idx = 0;
        uint8_t parity_idx = 0;
        for (uint8_t i = 0; i < UDP_RS_TOTAL_CHUNKS; i++) {
            uint8_t chunk_id;
            uint8_t chunk_type;
            const uint8_t* src;

            if ((i % 2 == 0) && (data_idx < UDP_RS_DATA_CHUNKS)) {
                chunk_id = data_idx;
                chunk_type = 0;
                src = dataBuf + data_idx * UDP_CHUNK_SIZE;
                data_idx++;
            } else {
                chunk_id = UDP_RS_DATA_CHUNKS + parity_idx;
                chunk_type = 1;
                src = parityBuf + parity_idx * UDP_CHUNK_SIZE;
                parity_idx++;
            }

            if (!send_udp_chunk(sock, &_dest_addr, _frame_id, block,
                                chunk_id, chunk_type, totalChunks,
                                src, UDP_CHUNK_SIZE)) {
                dropped++;
            }
            taskYIELD();
        }
    }

    if (dropped > 0) {
        ESP_LOGW(TAG, "Frame #%u: %u/%u chunks dropped",
                 _frame_id, dropped, totalChunks);
    }

    free(dataBuf);
    free(parityBuf);
    return true;
}

void UDPStream::streamTaskFn(void* arg)
{
    auto* self = static_cast<UDPStream*>(arg);
    int sock = -1;

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(self->TAG, "Failed to create UDP socket");
        self->_active = false;
        vTaskDelete(NULL);
        return;
    }

    int broadcast_enable = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) < 0) {
        ESP_LOGW(self->TAG, "Failed to enable SO_BROADCAST, continuing anyway");
    }

    ESP_LOGI(self->TAG, "UDP stream task started");
    esp_log_level_set(self->TAG, ESP_LOG_INFO);

    uint8_t helo[] = {0x48, 0x45, 0x4C, 0x4F};
    int helo_sent = sendto(sock, helo, sizeof(helo), 0,
                           (const struct sockaddr*)&self->_dest_addr,
                           sizeof(self->_dest_addr));
    ESP_LOGI(self->TAG, "HELO probe: sendto() returned %d (errno=%d)", helo_sent, helo_sent < 0 ? errno : 0);

    while (self->_active) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(self->TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        self->sendFrame(sock, fb);
        esp_camera_fb_return(fb);
        self->_frame_id++;

        int fps = CONFIG_UDP_STREAM_FPS;
        if (fps <= 0) fps = 30;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000 / fps));
    }

    ESP_LOGI(self->TAG, "UDP stream task exiting");
    if (sock >= 0) {
        close(sock);
    }
    self->_task = nullptr;
    vTaskDelete(NULL);
}
