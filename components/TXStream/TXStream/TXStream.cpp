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
    size_t offset = 0;
    uint8_t total_chunks = (len + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;
    printf("Sending frame %d with %d chunks (%d bytes total)\n", frame_id, total_chunks, len);
    vTaskDelay(pdMS_TO_TICKS(1)); // Wait slightly to let the buffer fill before transmitting

    while (offset < len) {
        size_t chunk_len = len - offset > MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : len - offset;
        uint8_t buffer[1500] = {0}; 

        // 802.11 Data Frame header construction
        wifi_ieee80211_data_hdr_t *hdr = (wifi_ieee80211_data_hdr_t *)buffer;

        // Frame Control for regular Data frame
        hdr->frame_control[0] = 0x08;  // Data frame, To DS = 1 (changed from 0x88)
        hdr->frame_control[1] = 0x00;  // No special flags

        // Duration field - set to 0
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

        // LLC/SNAP header 
        uint8_t *llc_snap = buffer + sizeof(wifi_ieee80211_data_hdr_t);
        *llc_snap++ = 0xAA;  // DSAP
        *llc_snap++ = 0xAA;  // SSAP
        *llc_snap++ = 0x03;  // Control
        *llc_snap++ = 0x00;  // OUI byte 1
        *llc_snap++ = 0x00;  // OUI byte 2
        *llc_snap++ = 0x00;  // OUI byte 3
        *llc_snap++ = 0x88;  // EtherType high byte (custom)
        *llc_snap++ = 0xB5;  // EtherType low byte (custom)

        // Custom header for frame reconstruction
        uint8_t *custom_hdr = llc_snap;
        memcpy(custom_hdr, vendor_oui, 3);          // OUI for identification
        custom_hdr += 3;
        *custom_hdr++ = frame_id;                   // Frame ID
        *custom_hdr++ = offset / MAX_PAYLOAD_SIZE;  // Chunk ID
        *custom_hdr++ = total_chunks;               // Total chunks

        // Length field for this chunk
        *custom_hdr++ = (chunk_len >> 8) & 0xFF;    // Length high byte
        *custom_hdr++ = chunk_len & 0xFF;           // Length low byte

        // Copy JPEG data
        memcpy(custom_hdr, jpeg + offset, chunk_len);
        custom_hdr += chunk_len;

        // No parity bit calculation

        size_t frame_len = custom_hdr - buffer;

        // No FCS calculation, reduce the frame length accordingly
        esp_err_t result = esp_wifi_80211_tx(WIFI_IF_STA, buffer, frame_len, false);
        if (result != ESP_OK) {
            printf("TX failed: %s\n", esp_err_to_name(result));
        }

        offset += chunk_len;
    }
    printf("Sent Frame %d\n", frame_id);

    frame_id++;
}