#include "wifiManager.hpp"

static auto WIFI_MANAGER_TAG = "[WIFI_MANAGER]";

//Start Receiver Vars
#define MAX_CHUNKS 512
#define VENDOR_OUI          {0xAC,0xDE,0x47}
static const uint8_t vendor_oui[3] = VENDOR_OUI;
#define MAX_PAYLOAD_SIZE 200
#define MAX_FRAME_SIZE (10*1024)
#define UART_PORT UART_NUM_0

static uint8_t frame_buf[MAX_FRAME_SIZE];
static uint8_t current_frame_id = 0xFF;
static uint8_t expected_chunks = 0;
static uint8_t received_chunks = 0;
static uint8_t chunk_map[MAX_CHUNKS / 8];

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

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
   * happened. */
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

    // we need to update the config after every credential change
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

// Add this method to the WiFiManager class
void WiFiManager::setJpegFrameCallback(JpegFrameCallback callback) {
    g_jpegFrameCallback = callback;
}
// Update your sniffer_cb function
static void sniffer_cb(void *buf, wifi_promiscuous_pkt_type_t t)
{
    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
    const uint8_t *payload = ppkt->payload;
    if ((payload[0] & 0xFC) != 0xD0) return;
    const uint8_t *action = payload + 24;
    if (action[0] != 127) return;
    if (memcmp(&action[1], vendor_oui, 3) != 0) return;
    uint8_t frame_id = action[4];
    uint8_t chunk_id = action[5];
    uint8_t total_chunks = action[6];
    const uint8_t *jpeg_data = &action[7];
    int jpeg_len = ppkt->rx_ctrl.sig_len - (jpeg_data - payload);
    
    if (frame_id != current_frame_id) {
        current_frame_id = frame_id;
        expected_chunks = total_chunks;
        received_chunks = 0;
        memset(frame_buf, 0, sizeof(frame_buf));
        memset(chunk_map, 0, sizeof(chunk_map));
    }
    
    if (chunk_id < expected_chunks) {
        if (!(chunk_map[chunk_id / 8] & (1 << (chunk_id % 8)))) {
            chunk_map[chunk_id / 8] |= (1 << (chunk_id % 8));
            memcpy(frame_buf + chunk_id * MAX_PAYLOAD_SIZE, jpeg_data, jpeg_len);
            received_chunks++;
        }
    }
    
    if (received_chunks == expected_chunks && all_chunks_received(expected_chunks)) {
        uint16_t total_length = expected_chunks * MAX_PAYLOAD_SIZE;
        // Trim excess to JPEG end marker (FFD9)
        while (total_length > 2 && !(frame_buf[total_length - 2] == 0xFF && frame_buf[total_length - 1] == 0xD9)) {
            total_length--;
        }
        printf("Received complete JPEG (%d chunks, %d bytes)\n", expected_chunks, total_length);
        
        // Call the provide_jpeg_frame method via callback
        printf("At Callback \n");
        if (g_jpegFrameCallback) {
            printf("Running Callback \n");
            g_jpegFrameCallback(frame_buf, total_length);
            printf("End Callback \n");
        }
    }
}

void WiFiManager::Begin()
{
  #ifdef CONFIG_TX_MODE        // This switching bs doesn't work rn, figure it out later
    ESP_LOGI(WIFI_MANAGER_TAG, "Beginning TX Startup");
    esp_netif_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_channel(CONFIG_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_err_t err = esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);
    ESP_LOGI(WIFI_MANAGER_TAG, "TX started.");
  #endif

  #ifdef CONFIG_RX_MODE
    ESP_LOGI(WIFI_MANAGER_TAG, "Beginning RX Startup");
    esp_netif_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_NULL);
    esp_wifi_start();
    esp_wifi_set_channel(CONFIG_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE); // Left Eye 6 | Right Eye 13 || Babble 3
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(sniffer_cb)); // Make callback func in another component 
    esp_wifi_set_promiscuous(true);
    ESP_LOGI(WIFI_MANAGER_TAG, "RX started.");
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

