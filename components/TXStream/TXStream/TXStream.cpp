#include "TXStream.hpp"
#include <new>

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

// ---------------------------------------------------------------------------
// RS(8,4) encode a single RS block at the byte-position level.
//
// For each of the 1400 byte positions, collect one byte from each of the
// 8 data chunks and RS-encode into 4 parity bytes.
// ---------------------------------------------------------------------------
static void rs_encode_block(
    const uint8_t* dataBuf,    // FEC_RS_DATA_CHUNKS * MAX_PAYLOAD_SIZE bytes
    uint8_t* parityBuf)       // FEC_RS_PARITY_CHUNKS * MAX_PAYLOAD_SIZE bytes
{
    RS::ReedSolomon<FEC_RS_DATA_CHUNKS, FEC_RS_PARITY_CHUNKS> rs;
    uint8_t rsSrc[FEC_RS_DATA_CHUNKS];
    uint8_t rsEcc[FEC_RS_PARITY_CHUNKS];

    for (uint16_t i = 0; i < MAX_PAYLOAD_SIZE; i++) {
        // Collect one byte from each data chunk at position i
        for (uint8_t c = 0; c < FEC_RS_DATA_CHUNKS; c++) {
            rsSrc[c] = dataBuf[c * MAX_PAYLOAD_SIZE + i];
        }

        // RS encode: 8 data bytes -> 4 parity bytes
        rs.EncodeBlock(rsSrc, rsEcc);

        // Store parity bytes in parity chunks at position i
        for (uint8_t p = 0; p < FEC_RS_PARITY_CHUNKS; p++) {
            parityBuf[p * MAX_PAYLOAD_SIZE + i] = rsEcc[p];
        }
    }
}

void TXStream::send_jpeg_frame(const uint8_t *jpeg, size_t len)
{
    static uint8_t frame_id = 0;
    static uint16_t seq = 0;

    if (!jpeg || len == 0) return;

    // Calculate how many data chunks the JPEG needs
    uint8_t numDataChunks = (uint8_t)((len + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE);
    if (numDataChunks == 0) numDataChunks = 1;

    // Round up to fill complete RS blocks (multiple of FEC_RS_DATA_CHUNKS)
    uint8_t numRsBlocks = (numDataChunks + FEC_RS_DATA_CHUNKS - 1) / FEC_RS_DATA_CHUNKS;
    if (numRsBlocks > FEC_MAX_RS_BLOCKS) numRsBlocks = FEC_MAX_RS_BLOCKS;

    printf("Sending frame %d with %d data chunks, %d RS blocks (%d bytes total)\n",
           frame_id, numDataChunks, numRsBlocks, (int)len);

    // Per-RS-block buffers
    uint8_t* dataBuf = new (std::nothrow) uint8_t[FEC_RS_DATA_CHUNKS * MAX_PAYLOAD_SIZE];
    uint8_t* parityBuf = new (std::nothrow) uint8_t[FEC_RS_PARITY_CHUNKS * MAX_PAYLOAD_SIZE];

    if (!dataBuf || !parityBuf) {
        printf("FEC: failed to allocate buffers\n");
        if (dataBuf) delete[] dataBuf;
        if (parityBuf) delete[] parityBuf;
        return;
    }

    // Pointer into the JPEG data for each RS block
    const uint8_t* jpegPtr = jpeg;
    size_t jpegRemaining = len;

    for (uint8_t block = 0; block < numRsBlocks; block++) {
        // Clear data buffer and copy JPEG data for this RS block
        memset(dataBuf, 0, FEC_RS_DATA_CHUNKS * MAX_PAYLOAD_SIZE);

        size_t blockDataSize = FEC_RS_DATA_CHUNKS * MAX_PAYLOAD_SIZE;
        size_t copySize = (jpegRemaining < blockDataSize) ? jpegRemaining : blockDataSize;
        memcpy(dataBuf, jpegPtr, copySize);

        jpegPtr += copySize;
        jpegRemaining -= copySize;

        // RS encode this block: produce parity chunks
        memset(parityBuf, 0, FEC_RS_PARITY_CHUNKS * MAX_PAYLOAD_SIZE);
        rs_encode_block(dataBuf, parityBuf);

        // Interleaved chunk transmission: D0, P0, D1, P1, D2, P2, D3, P3, P4, P5, P6, P7
        // Alternating data/parity spreads losses across both types so bursts are recoverable.
        {
            uint8_t data_idx = 0;
            uint8_t parity_idx = 0;
            for (uint8_t i = 0; i < FEC_RS_TOTAL_CHUNKS; i++) {
                uint8_t chunk_id;
                uint8_t chunk_type;
                const uint8_t* src;

                if ((i % 2 == 0) && (data_idx < FEC_RS_DATA_CHUNKS)) {
                    chunk_id = data_idx;
                    chunk_type = 0;
                    src = dataBuf + data_idx * MAX_PAYLOAD_SIZE;
                    data_idx++;
                } else {
                    chunk_id = FEC_RS_DATA_CHUNKS + parity_idx;
                    chunk_type = 1;
                    src = parityBuf + parity_idx * MAX_PAYLOAD_SIZE;
                    parity_idx++;
                }

                uint8_t buffer[1500] = {0};
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
                *custom_hdr++ = block;
                *custom_hdr++ = chunk_id;
                *custom_hdr++ = FEC_RS_TOTAL_CHUNKS;
                *custom_hdr++ = chunk_type;
                *custom_hdr++ = (MAX_PAYLOAD_SIZE >> 8) & 0xFF;
                *custom_hdr++ = MAX_PAYLOAD_SIZE & 0xFF;

                memcpy(custom_hdr, src, MAX_PAYLOAD_SIZE);
                custom_hdr += MAX_PAYLOAD_SIZE;

                size_t frame_len = custom_hdr - buffer;
                esp_err_t result = esp_wifi_80211_tx(WIFI_IF_STA, buffer, frame_len, false);
                if (result == ESP_ERR_NO_MEM) {
                    for (int retry = 0; retry < 20; retry++) {
                        vTaskDelay(pdMS_TO_TICKS(2));
                        result = esp_wifi_80211_tx(WIFI_IF_STA, buffer, frame_len, false);
                        if (result == ESP_OK) break;
                    }
                }
                if (result != ESP_OK) {
                    printf("TX failed chunk %d: %s\n", chunk_id, esp_err_to_name(result));
                }
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }

        printf("  RS block %d: %d data + %d parity chunks sent\n",
               block, FEC_RS_DATA_CHUNKS, FEC_RS_PARITY_CHUNKS);
    }

    printf("Sent Frame %d (FEC RS(8,4)\n", frame_id);
    frame_id++;

    delete[] dataBuf;
    delete[] parityBuf;
}
