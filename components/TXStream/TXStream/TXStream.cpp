#include "TXStream.hpp"

TXStream::TXStream(const int WIFI_CHANNEL)
    : _wifi_channel(WIFI_CHANNEL)
{
    // Constructor - initialize member variables
}

const char* const ETVR_HEADER = "\xFF\xA0";
const char* const ETVR_HEADER_FRAME = "\xFF\xA1";

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
            
            // Don't forget to return the frame buffer!
            esp_camera_fb_return(fb);
        }
        
        // Add a small delay to prevent overwhelming the system
        vTaskDelay(pdMS_TO_TICKS(1000/500));
    }
}


void TXStream::send_jpeg_frame(const uint8_t *jpeg, size_t len)
{
    static uint8_t frame_id = 0;
    static uint16_t seq = 0;  // Add sequence number tracking
    size_t offset = 0;
    uint8_t total_chunks = (len + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;
    printf("Sending frame %d with %d chunks (%d bytes total)\n", frame_id, total_chunks, len);

    while (offset < len) {
        size_t chunk_len = len - offset > MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : len - offset;
        uint8_t buffer[256] = {0};
        
        // Proper 802.11 MAC header construction
        wifi_ieee80211_hdr_t *hdr = (wifi_ieee80211_hdr_t *)buffer;
        
        hdr->frame_control[0] = FRAME_CONTROL;  // 0xD0
        hdr->frame_control[1] = 0x00;
        
        // Duration field (bytes 2-3) - set to 0
        hdr->duration[0] = 0x00;
        hdr->duration[1] = 0x00;
        
        // Address fields
        memset(hdr->addr1, 0xFF, 6);                    // Destination: broadcast
        esp_wifi_get_mac(WIFI_IF_STA, hdr->addr2);      // Source MAC
        memcpy(hdr->addr3, hdr->addr2, 6);              // BSSID = Source MAC
        
        // Sequence control
        hdr->seq_ctrl[0] = (seq & 0x0F) << 4;
        hdr->seq_ctrl[1] = (seq >> 4);
        seq = (seq + 1) & 0x0FFF;

        // Vendor-specific Action Frame payload
        uint8_t *payload = buffer + sizeof(wifi_ieee80211_hdr_t);
        *payload++ = 127;                           // Action category (vendor specific)
        memcpy(payload, vendor_oui, 3);            // OUI
        payload += 3;
        *payload++ = frame_id;                     // Frame ID
        *payload++ = offset / MAX_PAYLOAD_SIZE;    // Chunk ID
        *payload++ = total_chunks;                 // Total chunks
        
        // Copy JPEG data
        memcpy(payload, jpeg + offset, chunk_len);
        payload += chunk_len;

        size_t frame_len = payload - buffer;
        
        //printf("Sending chunk %d/%d (len=%d, frame_len=%d)\n", 
        //       (int)(offset / MAX_PAYLOAD_SIZE), total_chunks-1, (int)chunk_len, (int)frame_len);

        esp_err_t result = esp_wifi_80211_tx(WIFI_IF_STA, buffer, frame_len, false);
        if (result != ESP_OK) {
            printf("TX failed: %s\n", esp_err_to_name(result));
        }

        offset += chunk_len;
        vTaskDelay(pdMS_TO_TICKS(0.1)); // Slightly longer delay to avoid congestion
    }
    printf("Sent Frame");
    
    frame_id++;
}
    