#include "TXStream.hpp"

TXStream::TXStream(const int WIFI_CHANNEL) // Constructor
    : _wifi_channel(WIFI_CHANNEL)
{

}

void TXStream::startStream()
{
    while (true)
    {
        fb = esp_camera_fb_get();

        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed");
            response = ESP_FAIL;
        }
        else
        {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
            TXStream::send_jpeg_frame(_jpg_buf, _jpg_buf_len);
            
            esp_camera_fb_return(fb);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000/500));
    }
}

void TXStream::send_jpeg_frame(const uint8_t *jpeg, size_t len)
{
    static uint8_t frame_id = 0;
    static uint16_t seq = 0;

    // Repeat frame transmission 3 times
    for (int repeat = 0; repeat < 2; ++repeat) {
        size_t offset = 0;
        uint8_t total_chunks = (len + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;
        printf("Sending frame %d (attempt %d) with %d chunks (%d bytes total)\n",
               frame_id, repeat + 1, total_chunks, len);

        while (offset < len) {
            size_t chunk_len = len - offset > MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : len - offset;
            uint8_t buffer[1500] = {0}; 

            // 802.11 Data Frame header construction
            wifi_ieee80211_data_hdr_t *hdr = (wifi_ieee80211_data_hdr_t *)buffer;

            hdr->frame_control[0] = 0x08;
            hdr->frame_control[1] = 0x00;

            hdr->duration[0] = 0x00;
            hdr->duration[1] = 0x00;

            memset(hdr->addr1, 0xFF, 6);
            esp_wifi_get_mac(WIFI_IF_STA, hdr->addr2);
            memcpy(hdr->addr3, hdr->addr2, 6);

            hdr->seq_ctrl[0] = (seq & 0x0F) << 4;
            hdr->seq_ctrl[1] = (seq >> 4);
            seq = (seq + 1) & 0x0FFF;

            uint8_t *llc_snap = buffer + sizeof(wifi_ieee80211_data_hdr_t);
            *llc_snap++ = 0xAA;
            *llc_snap++ = 0xAA;
            *llc_snap++ = 0x03;
            *llc_snap++ = 0x00;
            *llc_snap++ = 0x00;
            *llc_snap++ = 0x00;
            *llc_snap++ = 0x88;
            *llc_snap++ = 0xB5;

            uint8_t *custom_hdr = llc_snap;
            memcpy(custom_hdr, vendor_oui, 3);
            custom_hdr += 3;
            *custom_hdr++ = frame_id;
            *custom_hdr++ = offset / MAX_PAYLOAD_SIZE;
            *custom_hdr++ = total_chunks;
            *custom_hdr++ = (chunk_len >> 8) & 0xFF;
            *custom_hdr++ = chunk_len & 0xFF;

            memcpy(custom_hdr, jpeg + offset, chunk_len);
            custom_hdr += chunk_len;

            size_t frame_len = custom_hdr - buffer;

            esp_err_t result = esp_wifi_80211_tx(WIFI_IF_STA, buffer, frame_len, false);
            if (result != ESP_OK) {
                printf("TX failed: %s\n", esp_err_to_name(result));
            }

            offset += chunk_len;
        }
    }

    printf("Sent Frame %d (3x)\n", frame_id);
    frame_id++;
}
