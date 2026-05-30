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

static fec_t* s_fec = nullptr;

static void rs_encode_block(
    const uint8_t* dataBuf,
    uint8_t* parityBuf)
{
    const gf* src[FEC_RS_DATA_CHUNKS];
    gf* dst[FEC_RS_PARITY_CHUNKS];
    unsigned block_nums[FEC_RS_PARITY_CHUNKS];

    for (uint8_t i = 0; i < FEC_RS_DATA_CHUNKS; i++)
        src[i] = dataBuf + i * MAX_PAYLOAD_SIZE;

    for (uint8_t i = 0; i < FEC_RS_PARITY_CHUNKS; i++) {
        dst[i] = parityBuf + i * MAX_PAYLOAD_SIZE;
        block_nums[i] = FEC_RS_DATA_CHUNKS + i;
    }

    fec_encode(s_fec, src, dst, block_nums, FEC_RS_PARITY_CHUNKS, MAX_PAYLOAD_SIZE);
}

static SemaphoreHandle_t s_tx_done_sem = nullptr;

static void tx_done_callback(const esp_80211_tx_info_t *tx_info)
{
    (void)tx_info;
    xSemaphoreGive(s_tx_done_sem);
}

void TXStream::send_jpeg_frame(const uint8_t *jpeg, size_t len)
{
    static uint8_t frame_id = 0;
    static uint16_t seq = 0;
    static bool tx_cb_registered = false;

    if (!tx_cb_registered) {
        s_tx_done_sem = xSemaphoreCreateBinary();
        if (s_tx_done_sem) {
            esp_wifi_register_80211_tx_cb(tx_done_callback);
        }
        tx_cb_registered = true;
    }

    if (!s_fec) {
        init_fec();
        s_fec = fec_new(FEC_RS_DATA_CHUNKS, FEC_RS_TOTAL_CHUNKS);
    }

    if (!jpeg || len == 0) return;

    // Calculate how many data chunks the JPEG needs
    uint8_t numDataChunks = (uint8_t)((len + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE);
    if (numDataChunks == 0) numDataChunks = 1;

    // Round up to fill complete RS blocks (multiple of FEC_RS_DATA_CHUNKS)
    uint8_t numRsBlocks = (numDataChunks + FEC_RS_DATA_CHUNKS - 1) / FEC_RS_DATA_CHUNKS;
    if (numRsBlocks > FEC_MAX_RS_BLOCKS) numRsBlocks = FEC_MAX_RS_BLOCKS;

    // Total chunks across all RS blocks (RX needs this to know when frame is complete)
    uint8_t totalFrameChunks = numRsBlocks * FEC_RS_TOTAL_CHUNKS;

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
                *custom_hdr++ = totalFrameChunks;
                *custom_hdr++ = chunk_type;
                *custom_hdr++ = (MAX_PAYLOAD_SIZE >> 8) & 0xFF;
                *custom_hdr++ = MAX_PAYLOAD_SIZE & 0xFF;

                memcpy(custom_hdr, src, MAX_PAYLOAD_SIZE);
                custom_hdr += MAX_PAYLOAD_SIZE;

                size_t frame_len = custom_hdr - buffer;
                // Clear stale completion signal before queuing
                xSemaphoreTake(s_tx_done_sem, 0);

                esp_err_t result = esp_wifi_80211_tx(WIFI_IF_STA, buffer, frame_len, false);

                if (result == ESP_OK) {
                    // Wait for hardware to finish transmitting (provides natural backpressure)
                    xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(50));
                } else if (result == ESP_ERR_NO_MEM) {
                    // Descriptor ring full — wait for current TX to drain, then retry
                    xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(50));
                    result = esp_wifi_80211_tx(WIFI_IF_STA, buffer, frame_len, false);
                    if (result == ESP_OK) {
                        xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(50));
                    }
                }
                if (result != ESP_OK) {
                    printf("TX failed chunk %d: %s\n", chunk_id, esp_err_to_name(result));
                }
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
