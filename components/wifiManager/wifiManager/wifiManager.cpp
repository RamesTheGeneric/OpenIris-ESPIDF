#include "wifiManager.hpp"

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

static uint8_t frame_buf[MAX_FRAME_SIZE];
static uint8_t current_frame_id = 0xFF;
static uint8_t expected_chunks = 0;
static uint8_t received_chunks = 0;
static uint8_t chunk_map[MAX_CHUNKS / 8];
static uint32_t frame_start_time = 0;

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

static inline bool all_chunks_received(uint8_t total_chunks) {
    for (int i = 0; i < total_chunks; ++i) {
        if (!(chunk_map[i / 8] & (1 << (i % 8)))) return false;
    }
    return true;
}

void WiFiManager::setJpegFrameCallback(JpegFrameCallback callback) {
    g_jpegFrameCallback = callback;
}

// Updated sniffer callback for data packets
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
    
    // Parse our custom header
    const uint8_t *custom_hdr = llc_snap + 8;
    
    // Verify OUI
    if (memcmp(custom_hdr, vendor_oui, 3) != 0) return;
    
    uint8_t frame_id = custom_hdr[3];
    uint8_t chunk_id = custom_hdr[4];
    uint8_t total_chunks = custom_hdr[5];
    uint16_t chunk_len = (custom_hdr[6] << 8) | custom_hdr[7];
    
    const uint8_t *jpeg_data = custom_hdr + 8;
    
    // Validate chunk length
    int max_possible_len = ppkt->rx_ctrl.sig_len - (jpeg_data - payload) - 4; // -4 for FCS
    if (chunk_len > max_possible_len || chunk_len > MAX_PAYLOAD_SIZE) {
        printf("Invalid chunk length: %d (max: %d)\n", chunk_len, max_possible_len);
        return;
    }
    
    // Handle new frame
    if (frame_id != current_frame_id) {
        if (current_frame_id != 0xFF) {
            printf("Frame %d incomplete (%d/%d chunks), starting frame %d\n", 
                   current_frame_id, received_chunks, expected_chunks, frame_id);
        }
        current_frame_id = frame_id;
        expected_chunks = total_chunks;
        received_chunks = 0;
        memset(frame_buf, 0, sizeof(frame_buf));
        memset(chunk_map, 0, sizeof(chunk_map));
        frame_start_time = esp_timer_get_time() / 1000; // ms
    }
    
    // Process chunk
    if (chunk_id < expected_chunks && chunk_id < MAX_CHUNKS) {
        if (!(chunk_map[chunk_id / 8] & (1 << (chunk_id % 8)))) {
            chunk_map[chunk_id / 8] |= (1 << (chunk_id % 8));
            
            // Calculate offset and ensure we don't overflow
            size_t offset = chunk_id * MAX_PAYLOAD_SIZE;
            if (offset + chunk_len <= MAX_FRAME_SIZE) {
                memcpy(frame_buf + offset, jpeg_data, chunk_len);
                received_chunks++;
                
                // Debug output for high chunk counts
                if (received_chunks % 10 == 0) {
                    printf("Received %d/%d chunks for frame %d\n", 
                           received_chunks, expected_chunks, frame_id);
                }
            } else {
                printf("Frame buffer overflow prevented (offset: %zu, len: %d)\n", offset, chunk_len);
            }
        }
    }
    
    // Check if frame is complete
    if (received_chunks == expected_chunks && all_chunks_received(expected_chunks)) {
        // Calculate actual frame size by finding JPEG end marker
        size_t total_length = 0;
        for (int i = expected_chunks - 1; i >= 0; i--) {
            size_t chunk_start = i * MAX_PAYLOAD_SIZE;
            size_t chunk_end = chunk_start + MAX_PAYLOAD_SIZE;
            if (chunk_end > MAX_FRAME_SIZE) chunk_end = MAX_FRAME_SIZE;
            
            // Look for JPEG end marker (0xFFD9) in reverse
            for (size_t j = chunk_end - 1; j >= chunk_start && j >= 1; j--) {
                if (frame_buf[j-1] == 0xFF && frame_buf[j] == 0xD9) {
                    total_length = j + 1;
                    break;
                }
            }
            if (total_length > 0) break;
        }
        
        if (total_length == 0) {
            total_length = expected_chunks * MAX_PAYLOAD_SIZE;
        }
        
        uint32_t frame_time = (esp_timer_get_time() / 1000) - frame_start_time;
        printf("Complete JPEG frame %d: %d chunks, %zu bytes, %lu ms\n", 
               frame_id, expected_chunks, total_length, frame_time);
        
        // Call the callback
        if (g_jpegFrameCallback) {
            g_jpegFrameCallback(frame_buf, total_length);
        }
        
        // Reset for next frame
        current_frame_id = 0xFF;
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
    esp_wifi_start();
    esp_wifi_set_channel(CONFIG_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_err_t err = esp_wifi_config_80211_tx_rate(WIFI_IF_STA,  WIFI_PHY_RATE_54M);
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
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(sniffer_cb));
    esp_wifi_set_promiscuous(true);
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