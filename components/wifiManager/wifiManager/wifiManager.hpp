#pragma once
#ifndef WIFIHANDLER_HPP
#define WIFIHANDLER_HPP

#include <string>
#include <cstring>
#include <algorithm>
#include <functional>
#include <StateManager.hpp>
#include <ProjectConfig.hpp>

#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_timer.h"

#define EXAMPLE_ESP_MAXIMUM_RETRY 3
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;

typedef std::function<void(uint8_t*, size_t)> JpegFrameCallback;

// 802.11 Data Frame header structure for RX parsing
typedef struct __attribute__((packed)) {
    uint8_t  frame_control[2];
    uint8_t  duration[2];
    uint8_t  addr1[6];      // Destination
    uint8_t  addr2[6];      // Source
    uint8_t  addr3[6];      // BSSID
    uint8_t  seq_ctrl[2];
} wifi_ieee80211_data_hdr_t;

// LLC/SNAP header structure
typedef struct __attribute__((packed)) {
    uint8_t  dsap;          // 0xAA
    uint8_t  ssap;          // 0xAA
    uint8_t  control;       // 0x03
    uint8_t  oui[3];        // 0x00, 0x00, 0x00
    uint16_t ethertype;     // Custom protocol identifier
} llc_snap_hdr_t;

// Custom protocol header for frame reconstruction
typedef struct __attribute__((packed)) {
    uint8_t  vendor_oui[3]; // Vendor OUI for identification
    uint8_t  frame_id;      // Frame identifier
    uint8_t  chunk_id;      // Chunk number
    uint8_t  total_chunks;  // Total chunks in frame
    uint16_t chunk_len;     // Length of this chunk
} custom_rx_hdr_t;

namespace WiFiManagerHelpers
{
  void event_handler(void *arg, esp_event_base_t event_base,
                     int32_t event_id, void *event_data);
}

class WiFiManager
{
private:
  uint8_t channel;
  std::shared_ptr<ProjectConfig> deviceConfig;
  QueueHandle_t eventQueue;
  StateManager *stateManager;
  wifi_init_config_t _wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
  wifi_config_t _wifi_cfg = {};

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;

  int8_t power;

  void SetCredentials(const char *ssid, const char *password);
  void ConnectWithHardcodedCredentials();
  void ConnectWithStoredCredentials();
  void SetupAccessPoint();
  void SetupWirelessTX();

public:
  WiFiManager(std::shared_ptr<ProjectConfig> deviceConfig, QueueHandle_t eventQueue, StateManager *stateManager);
  void Begin();
  void setJpegFrameCallback(JpegFrameCallback callback);
  
  // Additional utility functions for data packet mode
  static bool validateDataPacket(const uint8_t *packet, size_t len);
  static void resetFrameBuffer();
  static void printFrameStats();
};

#endif