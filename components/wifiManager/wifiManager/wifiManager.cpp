#include "wifiManager.hpp"
#include "rs.hpp"
#include <new>

static auto WIFI_MANAGER_TAG = "[WIFI_MANAGER]";

//Start Receiver Vars
#define MAX_CHUNKS 512
#define VENDOR_OUI          {0xAC,0xDE,0x47}
static const uint8_t vendor_oui[3] = VENDOR_OUI;
#define MAX_PAYLOAD_SIZE 1400
#define MAX_FRAME_SIZE (50*1024)
#define UART_PORT UART_NUM_0

// Custom EtherType
#define CUSTOM_ETHERTYPE 0x88B5

// RS(8,4) FEC parameters (must match TXStream)
#define FEC_RS_DATA_CHUNKS   8
#define FEC_RS_PARITY_CHUNKS 4
#define FEC_RS_TOTAL_CHUNKS  12
#define FEC_MAX_RS_BLOCKS    4

static uint8_t frame_buf[MAX_FRAME_SIZE];
static uint8_t current_frame_id = 0xFF;
static uint8_t expected_chunks = 0;
static uint8_t received_chunks = 0;
static uint8_t chunk_map[MAX_CHUNKS / 8];
static uint32_t frame_start_time = 0;

// Per-RS-block decode state
static uint8_t rs_block_received[FEC_MAX_RS_BLOCKS][FEC_RS_TOTAL_CHUNKS];
static uint8_t rs_block_count = 0;
static uint8_t frame_decoded = 0;  // Flag to prevent repeated decode of same frame

// Decode task: semaphore + handle
static SemaphoreHandle_t rs_decode_semaphore = nullptr;
static TaskHandle_t rs_decode_task_handle = nullptr;

static JpegFrameCallback g_jpegFrameCallback = nullptr;

//End Receiver Vars

void WiFiManagerHelpers::event_handler(void *arg, esp_event_base_t event_base,
                                       int32_t event_id, void *event_data)
{
  ESP_LOGI(WIFI_MANAGER_TAG, "Trying to connect, got event: %d", (int)event_id);
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    if (const auto err = esp_wifi_connect(); err != ESP_OK)
    {
      ESP_LOGI(WIFI_MANAGER_TAG, "esp_wifi_connect() failed: %s", esp_err_to_name(err));
    }
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
    {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(WIFI_MANAGER_TAG, "retry to connect to the AP");
    }
    else
    {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(WIFI_MANAGER_TAG, "connect to the AP fail");
  }

  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    const auto *event = static_cast<ip_event_got_ip_t *>(event_data);
    ESP_LOGI(WIFI_MANAGER_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

WiFiManager::WiFiManager(std::shared_ptr<ProjectConfig> deviceConfig, QueueHandle_t eventQueue, StateManager *stateManager) : deviceConfig(deviceConfig), eventQueue(eventQueue), stateManager(stateManager) {}

void WiFiManager::SetCredentials(const char *ssid, const char *password)
{
  memcpy(_wifi_cfg.sta.ssid, ssid, std::min(strlen(ssid), sizeof(_wifi_cfg.sta.ssid)));
  memcpy(_wifi_cfg.sta.password, password, std::min(strlen(password), sizeof(_wifi_cfg.sta.password)));
}

void WiFiManager::ConnectWithHardcodedCredentials()
{
  SystemEvent event = {EventSource::WIFI, WiFiState_e::WiFiState_ReadyToConnect};
  this->SetCredentials(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &_wifi_cfg));

  xQueueSend(this->eventQueue, &event, 10);
  esp_wifi_start();

  event.value = WiFiState_e::WiFiState_Connecting;
  xQueueSend(this->eventQueue, &event, 10);

  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE,
                                         pdFALSE,
                                         portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT)
  {
    ESP_LOGI(WIFI_MANAGER_TAG, "connected to ap SSID:%p password:%p",
             _wifi_cfg.sta.ssid, _wifi_cfg.sta.password);

    event.value = WiFiState_e::WiFiState_Connected;
    xQueueSend(this->eventQueue, &event, 10);
  }
  else if (bits & WIFI_FAIL_BIT)
  {
    ESP_LOGE(WIFI_MANAGER_TAG, "Failed to connect to SSID:%p, password:%p",
             _wifi_cfg.sta.ssid, _wifi_cfg.sta.password);

    event.value = WiFiState_e::WiFiState_Error;
    xQueueSend(this->eventQueue, &event, 10);
  }
  else
  {
    ESP_LOGE(WIFI_MANAGER_TAG, "UNEXPECTED EVENT");
  }
}

void WiFiManager::ConnectWithStoredCredentials()
{
  SystemEvent event = {EventSource::WIFI, WiFiState_e::WiFiState_ReadyToConnect};

  auto const networks = this->deviceConfig->getWifiConfigs();

  if (networks.empty())
  {
    event.value = WiFiState_e::WiFiState_Disconnected;
    xQueueSend(this->eventQueue, &event, 10);
    ESP_LOGE(WIFI_MANAGER_TAG, "No networks stored, cannot connect");
    return;
  }

  for (const auto& network : networks)
  {
    xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
    this->SetCredentials(network.ssid.c_str(), network.password.c_str());

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &_wifi_cfg));
    xQueueSend(this->eventQueue, &event, 10);

    esp_wifi_start();

    event.value = WiFiState_e::WiFiState_Connecting;
    xQueueSend(this->eventQueue, &event, 10);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT)
    {
      ESP_LOGI(WIFI_MANAGER_TAG, "connected to ap SSID:%p password:%p",
               _wifi_cfg.sta.ssid, _wifi_cfg.sta.password);

      event.value = WiFiState_e::WiFiState_Connected;
      xQueueSend(this->eventQueue, &event, 10);
      return;
    }
    ESP_LOGE(WIFI_MANAGER_TAG, "Failed to connect to SSID:%p, password:%p, trying next stored network",
             _wifi_cfg.sta.ssid, _wifi_cfg.sta.password);
  }

  event.value = WiFiState_e::WiFiState_Error;
  xQueueSend(this->eventQueue, &event, 10);
  ESP_LOGE(WIFI_MANAGER_TAG, "Failed to connect to all saved networks");
}

void WiFiManager::SetupAccessPoint()
{
  ESP_LOGI(WIFI_MANAGER_TAG, "Connection to stored credentials failed, starting AP");

  esp_netif_create_default_wifi_ap();
  wifi_init_config_t esp_wifi_ap_init_config = WIFI_INIT_CONFIG_DEFAULT();

  ESP_ERROR_CHECK(esp_wifi_init(&esp_wifi_ap_init_config));

  wifi_config_t ap_wifi_config = {
      .ap = {
          .ssid = CONFIG_AP_WIFI_SSID,
          .password = CONFIG_AP_WIFI_PASSWORD,
          .max_connection = 1,
      },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(WIFI_MANAGER_TAG, "AP started.");
}

// ---------------------------------------------------------------------------
// RS(8,4) decode a single RS block at the byte-position level.
//
// For each of the 1400 byte positions, collect available symbols from all
// received chunks (data + parity) and RS-decode to recover the 8 data bytes.
// If all 8 data chunks are present, copy directly without decode.
// ---------------------------------------------------------------------------
static bool rs_decode_block(uint8_t blockId, uint8_t* out, size_t* outLen)
{
    // Count how many data and parity chunks are present
    uint8_t dataPresent = 0;
    uint8_t parityPresent = 0;
    for (uint8_t c = 0; c < FEC_RS_DATA_CHUNKS; c++) {
        if (rs_block_received[blockId][c]) dataPresent++;
    }
    for (uint8_t p = 0; p < FEC_RS_PARITY_CHUNKS; p++) {
        if (rs_block_received[blockId][FEC_RS_DATA_CHUNKS + p]) parityPresent++;
    }

    // If all 8 data chunks are present, copy directly
    if (dataPresent >= FEC_RS_DATA_CHUNKS) {
        memcpy(out, frame_buf + blockId * FEC_RS_TOTAL_CHUNKS * MAX_PAYLOAD_SIZE,
               FEC_RS_DATA_CHUNKS * MAX_PAYLOAD_SIZE);
        *outLen = FEC_RS_DATA_CHUNKS * MAX_PAYLOAD_SIZE;
        return true;
    }

    // Need RS decode. Check we have enough parity to recover missing data.
    // RS(8,4) can correct up to 4 erasures. Missing data chunks = erasures.
    uint8_t missingData = FEC_RS_DATA_CHUNKS - dataPresent;
    if (missingData > parityPresent) {
        return false;
    }

    // RS(8,4) decode at each byte position
    // Only pass DATA erasure positions to the RS library (0..7).
    // Parity erasures are handled by the parityPresent check above.
    RS::ReedSolomon<FEC_RS_DATA_CHUNKS, FEC_RS_PARITY_CHUNKS> rs;
    uint8_t baseOffset = blockId * FEC_RS_TOTAL_CHUNKS * MAX_PAYLOAD_SIZE;

    for (uint16_t i = 0; i < MAX_PAYLOAD_SIZE; i++) {
        // Yield periodically to prevent task watchdog timeout
        if (i % 256 == 0 && i != 0) taskYIELD();

        // Build data buffer (8 bytes, one from each data chunk at position i)
        uint8_t dataBytes[FEC_RS_DATA_CHUNKS];
        // Build ECC buffer (4 bytes, one from each parity chunk at position i)
        uint8_t eccBytes[FEC_RS_PARITY_CHUNKS];
        // Build erasure list (only data positions 0..7)
        uint8_t erasePos[FEC_RS_DATA_CHUNKS];
        uint16_t eraseCount = 0;

        for (uint8_t c = 0; c < FEC_RS_DATA_CHUNKS; c++) {
            if (rs_block_received[blockId][c]) {
                dataBytes[c] = frame_buf[baseOffset + c * MAX_PAYLOAD_SIZE + i];
            } else {
                dataBytes[c] = 0;
                erasePos[eraseCount++] = c;
            }
        }

        for (uint8_t p = 0; p < FEC_RS_PARITY_CHUNKS; p++) {
            uint8_t parityIdx = FEC_RS_DATA_CHUNKS + p;
            if (rs_block_received[blockId][parityIdx]) {
                eccBytes[p] = frame_buf[baseOffset + parityIdx * MAX_PAYLOAD_SIZE + i];
            } else {
                eccBytes[p] = 0;
            }
        }

        // Decode using DecodeBlock: separate data and ECC buffers
        uint8_t decoded[FEC_RS_DATA_CHUNKS];
        if (eraseCount > 0) {
            int err = rs.DecodeBlock(dataBytes, eccBytes, decoded, erasePos, eraseCount);
            if (err != 0) {
                return false;
            }
        } else {
            memcpy(decoded, dataBytes, FEC_RS_DATA_CHUNKS);
        }

        // Write decoded bytes to output (one byte per data chunk at position i)
        for (uint8_t c = 0; c < FEC_RS_DATA_CHUNKS; c++) {
            out[c * MAX_PAYLOAD_SIZE + i] = decoded[c];
        }
    }

    *outLen = FEC_RS_DATA_CHUNKS * MAX_PAYLOAD_SIZE;
    return true;
}

// Check if enough chunks have been received in an RS block for decode
static inline bool rs_block_decodable(uint8_t blockId)
{
    uint8_t count = 0;
    for (uint8_t c = 0; c < FEC_RS_TOTAL_CHUNKS; c++) {
        if (rs_block_received[blockId][c]) count++;
    }
    return count >= FEC_RS_DATA_CHUNKS;
}

// Check if all RS blocks for the current frame are decodable
static inline bool all_rs_blocks_decodable()
{
    for (uint8_t b = 0; b < rs_block_count; b++) {
        if (!rs_block_decodable(b)) return false;
    }
    return true;
}

void WiFiManager::setJpegFrameCallback(JpegFrameCallback callback) {
    g_jpegFrameCallback = callback;
}

// ---------------------------------------------------------------------------
// RS decode task: runs on dedicated core, offloaded from WiFi task.
// Waits on semaphore, decodes frame, calls JPEG callback.
// ---------------------------------------------------------------------------
static void rs_decode_task_fn(void* arg)
{
    while (true) {
        if (xSemaphoreTake(rs_decode_semaphore, portMAX_DELAY) != pdTRUE) continue;

        uint8_t* decoded = new (std::nothrow) uint8_t[FEC_MAX_RS_BLOCKS * FEC_RS_DATA_CHUNKS * MAX_PAYLOAD_SIZE];
        if (!decoded) continue;
        memset(decoded, 0, FEC_MAX_RS_BLOCKS * FEC_RS_DATA_CHUNKS * MAX_PAYLOAD_SIZE);

        size_t totalDecodedLen = 0;
        bool decodeOk = true;

        for (uint8_t b = 0; b < rs_block_count; b++) {
            size_t blockLen = 0;
            if (!rs_decode_block(b, decoded + totalDecodedLen, &blockLen)) {
                decodeOk = false;
                break;
            }
            totalDecodedLen += blockLen;
        }

        if (decodeOk) {
            // Find JPEG end marker
            size_t total_length = 0;
            for (size_t i = totalDecodedLen - 1; i >= 1; i--) {
                if (decoded[i-1] == 0xFF && decoded[i] == 0xD9) {
                    total_length = i + 1;
                    break;
                }
            }
            if (total_length == 0) total_length = totalDecodedLen;

            ESP_LOGI(WIFI_MANAGER_TAG, "RS decode OK: %zu bytes, JPEG end at %zu, blocks=%d",
                     totalDecodedLen, total_length, rs_block_count);
            ESP_LOGI(WIFI_MANAGER_TAG, "JPEG header: 0x%02X%02X%02X%02X",
                     decoded[0], decoded[1], decoded[2], decoded[3]);

            if (g_jpegFrameCallback) {
                g_jpegFrameCallback(decoded, total_length);
            }
        } else {
            ESP_LOGW(WIFI_MANAGER_TAG, "RS decode FAILED: blocks=%d", rs_block_count);
        }

        delete[] decoded;
    }
}

// Updated sniffer callback for data packets with RS(8,4) FEC
static void sniffer_cb(void *buf, wifi_promiscuous_pkt_type_t t)
{
    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
    const uint8_t *payload = ppkt->payload;

    // Check if it's a regular data frame (frame control = 0x08, not 0x88)
    if ((payload[0] & 0xFC) != 0x08) return;

    // Skip to LLC/SNAP header (after 802.11 header)
    const uint8_t *llc_snap = payload + 24;

    // Verify LLC/SNAP header for our custom protocol
    if (llc_snap[0] != 0xAA || llc_snap[1] != 0xAA || llc_snap[2] != 0x03) return;
    if (llc_snap[3] != 0x00 || llc_snap[4] != 0x00 || llc_snap[5] != 0x00) return;

    // Check our custom EtherType
    uint16_t ethertype = (llc_snap[6] << 8) | llc_snap[7];
    if (ethertype != CUSTOM_ETHERTYPE) return;

    // Parse our custom header (11 bytes with FEC fields)
    const uint8_t *custom_hdr = llc_snap + 8;

    // Verify OUI
    if (memcmp(custom_hdr, vendor_oui, 3) != 0) return;

    uint8_t frame_id = custom_hdr[3];
    uint8_t rs_block_id = custom_hdr[4];
    uint8_t chunk_id = custom_hdr[5];
    uint8_t total_chunks = custom_hdr[6];
    uint8_t chunk_type = custom_hdr[7]; (void)chunk_type;
    uint16_t chunk_len = (custom_hdr[8] << 8) | custom_hdr[9];

    const uint8_t *jpeg_data = custom_hdr + 10;

    // Validate chunk length
    int max_possible_len = ppkt->rx_ctrl.sig_len - (jpeg_data - payload);
    if (chunk_len > max_possible_len || chunk_len > MAX_PAYLOAD_SIZE) {
        return;
    }

    // Validate RS block ID
    if (rs_block_id >= FEC_MAX_RS_BLOCKS) {
        return;
    }

    // Handle new frame
    if (frame_id != current_frame_id) {
        current_frame_id = frame_id;
        expected_chunks = total_chunks;
        received_chunks = 0;
        rs_block_count = 0;
        frame_decoded = 0;
        memset(frame_buf, 0, sizeof(frame_buf));
        memset(chunk_map, 0, sizeof(chunk_map));
        memset(rs_block_received, 0, sizeof(rs_block_received));
        frame_start_time = esp_timer_get_time() / 1000; // ms
    }

    // Process chunk
    if (chunk_id < FEC_RS_TOTAL_CHUNKS) {
        if (!rs_block_received[rs_block_id][chunk_id]) {
            rs_block_received[rs_block_id][chunk_id] = 1;

            // Calculate offset in frame buffer
            size_t offset = (rs_block_id * FEC_RS_TOTAL_CHUNKS + chunk_id) * MAX_PAYLOAD_SIZE;
            if (offset + chunk_len <= MAX_FRAME_SIZE) {
                memcpy(frame_buf + offset, jpeg_data, chunk_len);
                received_chunks++;

                // Track RS block count
                if (rs_block_id + 1 > rs_block_count)
                    rs_block_count = rs_block_id + 1;
            } else {
                // Frame buffer overflow prevented - silent fail
            }
        }
    }

    // Signal decode task when frame is ready (only once per frame)
    if (!frame_decoded && all_rs_blocks_decodable()) {
        frame_decoded = 1;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(rs_decode_semaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void WiFiManager::Begin()
{
  #ifdef CONFIG_TX_MODE
    ESP_LOGI(WIFI_MANAGER_TAG, "Beginning TX Startup");
    esp_netif_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_err_t err = esp_wifi_config_80211_tx_rate(WIFI_IF_STA,  WIFI_PHY_RATE_2M_L);
    esp_wifi_start();
    esp_wifi_set_channel(CONFIG_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    ESP_LOGI(WIFI_MANAGER_TAG, "TX started on channel %d", CONFIG_WIFI_CHANNEL);
  #endif

  #ifdef CONFIG_RX_MODE
    ESP_LOGI(WIFI_MANAGER_TAG, "Beginning RX Startup");
    esp_netif_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_NULL);
    esp_wifi_start();
    esp_wifi_set_channel(CONFIG_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
        // Configure promiscuous filter to only receive data packets
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(sniffer_cb));
    esp_wifi_set_promiscuous(true);

    // Create RS decode task (offloaded from WiFi task to prevent heap corruption)
    rs_decode_semaphore = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(rs_decode_task_fn, "rs_decode", 4096, nullptr, 5, &rs_decode_task_handle, 1);

    ESP_LOGI(WIFI_MANAGER_TAG, "RX started on channel %d", CONFIG_WIFI_CHANNEL);
  #endif
    
  #if !defined(CONFIG_TX_MODE) && !defined(CONFIG_RX_MODE)
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  auto netif = esp_netif_create_default_wifi_sta();

  wifi_init_config_t esp_wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&esp_wifi_init_config));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &WiFiManagerHelpers::event_handler,
                                                      nullptr,
                                                      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &WiFiManagerHelpers::event_handler,
                                                      nullptr,
                                                      &instance_got_ip));

  _wifi_cfg = {};
  _wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WEP;
  _wifi_cfg.sta.pmf_cfg.capable = true;
  _wifi_cfg.sta.pmf_cfg.required = false;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  ESP_LOGI(WIFI_MANAGER_TAG, "Beginning setup");
  const auto hasHardcodedCredentials = strlen(CONFIG_WIFI_SSID) > 0;
  if (hasHardcodedCredentials)
  {
    ESP_LOGI(WIFI_MANAGER_TAG, "Detected hardcoded credentials, trying them out");
    this->ConnectWithHardcodedCredentials();
  }

  if (this->stateManager->GetWifiState() != WiFiState_e::WiFiState_Connected || !hasHardcodedCredentials)
  {
    ESP_LOGI(WIFI_MANAGER_TAG, "Hardcoded credentials failed or missing, trying stored credentials");
    xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
    this->ConnectWithStoredCredentials();
  }

  if (this->stateManager->GetWifiState() != WiFiState_e::WiFiState_Connected)
  {
    ESP_LOGI(WIFI_MANAGER_TAG, "Stored netoworks failed or hardcoded credentials missing, starting AP");
    xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
    esp_netif_destroy(netif);
    this->SetupAccessPoint();
  }
  #endif
}