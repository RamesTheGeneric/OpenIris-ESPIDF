#include "UVCStream.hpp"
constexpr int UVC_MAX_FRAMESIZE_SIZE(75 * 1024);

static const char *UVC_STREAM_TAG = "[UVC DEVICE]";

namespace UVCStreamHelpers
{
  uint16_t frameWidth = 0;
  uint16_t frameHeight = 0;
}

static esp_err_t UVCStreamHelpers::camera_start_cb(uvc_format_t format, int width, int height, int rate, void *cb_ctx)
{
  (void)cb_ctx;
  frameWidth = width;
  frameHeight = height;
  ESP_LOGI(UVC_STREAM_TAG, "Camera Start");
  ESP_LOGI(UVC_STREAM_TAG, "Format: %d, width: %d, height: %d, rate: %d", format, width, height, rate);
  #ifndef CONFIG_RX_MODE
  framesize_t frame_size = FRAMESIZE_QQVGA;
  #endif

  if (format != UVC_FORMAT_JPEG)
  {
    ESP_LOGE(UVC_STREAM_TAG, "Only support MJPEG format");
    return ESP_ERR_NOT_SUPPORTED;
  }

  if (width == frameWidth && height == frameHeight)
  {
    #ifndef CONFIG_RX_MODE
    frame_size = FRAMESIZE_QQVGA;
    #endif
  }
  else
  {
    ESP_LOGE(UVC_STREAM_TAG, "Unsupported frame size %dx%d", width, height);
    return ESP_ERR_NOT_SUPPORTED;
  }
  #ifndef CONFIG_RX_MODE
  cameraHandler->setCameraResolution(frame_size);
  cameraHandler->resetCamera(false);
  #endif

  constexpr SystemEvent event = {EventSource::STREAM, StreamState_e::Stream_ON};
  xQueueSend(eventQueue, &event, 10);

  return ESP_OK;
}

static void UVCStreamHelpers::camera_stop_cb(void *cb_ctx)
{
  (void)cb_ctx;
  #ifndef CONFIG_RX_MODE
  if (s_fb.cam_fb_p)
  {
    esp_camera_fb_return(s_fb.cam_fb_p);
    s_fb.cam_fb_p = nullptr;
  }
  #endif

  constexpr SystemEvent event = {EventSource::STREAM, StreamState_e::Stream_OFF};
  xQueueSend(eventQueue, &event, 10);
}

static uvc_fb_t *UVCStreamHelpers::camera_fb_get_cb(void *cb_ctx)
{
  ESP_LOGW(UVC_STREAM_TAG, "FB_CB");
  (void)cb_ctx;
  #ifndef CONFIG_RX_MODE
  s_fb.cam_fb_p = esp_camera_fb_get();
  if (!s_fb.cam_fb_p)
  {
    ESP_LOGE(UVC_STREAM_TAG, "No camera FB");
    return nullptr;
  }
  s_fb.uvc_fb.buf = s_fb.cam_fb_p->buf;
  s_fb.uvc_fb.len = s_fb.cam_fb_p->len;
  s_fb.uvc_fb.width = s_fb.cam_fb_p->width;
  s_fb.uvc_fb.height = s_fb.cam_fb_p->height;
  s_fb.uvc_fb.format = UVC_FORMAT_JPEG; // we gotta make sure we're ALWAYS using JPEG
  s_fb.uvc_fb.timestamp = s_fb.cam_fb_p->timestamp;
  #else
  if (xSemaphoreTake(frame_ready_sem, pdMS_TO_TICKS(100)) != pdTRUE)
  {
    return nullptr; // Timeout - no frame available
  }
  // Get Self Contained jpeg frame here
  s_fb.uvc_fb.buf = jpeg_s_fb.buf;
  s_fb.uvc_fb.len = jpeg_s_fb.len;
  s_fb.uvc_fb.width = frameWidth;
  s_fb.uvc_fb.height = frameHeight;
  s_fb.uvc_fb.format = UVC_FORMAT_JPEG; // we gotta make sure we're ALWAYS using JPEG
  s_fb.uvc_fb.timestamp = static_cast<timeval>(esp_timer_get_time());
  #endif

  if (s_fb.uvc_fb.len > UVC_MAX_FRAMESIZE_SIZE)
  {
    ESP_LOGE(UVC_STREAM_TAG, "Frame size %d is larger than max frame size %d", s_fb.uvc_fb.len, UVC_MAX_FRAMESIZE_SIZE);
    #ifndef CONFIG_RX_MODE
    esp_camera_fb_return(s_fb.cam_fb_p);
    #endif
    return nullptr;
  }

  return &s_fb.uvc_fb;
}

static void UVCStreamHelpers::camera_fb_return_cb(uvc_fb_t *fb, void *cb_ctx)
{
  (void)cb_ctx;
  assert(fb == &s_fb.uvc_fb);
  #ifndef CONFIG_RX_MODE
  esp_camera_fb_return(s_fb.cam_fb_p);
  #endif
}

esp_err_t UVCStreamManager::setup()
{
  ESP_LOGI(UVC_STREAM_TAG, "Setting up UVC Stream");

  #ifdef CONFIG_RX_MODE
  UVCStreamHelpers::frame_ready_sem = xSemaphoreCreateBinary();
  if (UVCStreamHelpers::frame_ready_sem == nullptr)
  {
    ESP_LOGE(UVC_STREAM_TAG, "Failed to create frame ready semaphore");
    return ESP_FAIL;
  }
  #endif

  uvc_buffer = static_cast<uint8_t *>(malloc(UVC_MAX_FRAMESIZE_SIZE));
  if (uvc_buffer == nullptr)
  {
    ESP_LOGE(UVC_STREAM_TAG, "Allocating buffer for UVC Device failed");
    return ESP_FAIL;
  }

  uvc_device_config_t config = {
      .uvc_buffer = uvc_buffer,
      .uvc_buffer_size = UVC_MAX_FRAMESIZE_SIZE,
      .start_cb = UVCStreamHelpers::camera_start_cb,
      .fb_get_cb = UVCStreamHelpers::camera_fb_get_cb,
      .fb_return_cb = UVCStreamHelpers::camera_fb_return_cb,
      .stop_cb = UVCStreamHelpers::camera_stop_cb,
  };

  esp_err_t ret = uvc_device_config(0, &config);
  if (ret != ESP_OK)
  {
    ESP_LOGE(UVC_STREAM_TAG, "Configuring UVC Device failed: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(UVC_STREAM_TAG, "Configured UVC Device");

  ESP_LOGI(UVC_STREAM_TAG, "Initializing UVC Device");
  ret = uvc_device_init();
  if (ret != ESP_OK)
  {
    ESP_LOGE(UVC_STREAM_TAG, "Initializing UVC Device failed: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(UVC_STREAM_TAG, "Initialized UVC Device");

  return ESP_OK;
}

#ifdef CONFIG_RX_MODE
void UVCStreamManager::provide_jpeg_frame(uint8_t *jpeg_data, size_t jpeg_len)
{
  if (!jpeg_data || jpeg_len == 0 || jpeg_len > UVC_MAX_FRAMESIZE_SIZE)
  {
    ESP_LOGW(UVC_STREAM_TAG, "Invalid JPEG frame input");
    return;
  }

  // Free any previously allocated buffer
  if (UVCStreamHelpers::jpeg_s_fb.buf != nullptr)
  {
    free(UVCStreamHelpers::jpeg_s_fb.buf);
    UVCStreamHelpers::jpeg_s_fb.buf = nullptr;
    UVCStreamHelpers::jpeg_s_fb.len = 0;
  }

  // Allocate and copy JPEG data into a new buffer
  uint8_t *new_buf = (uint8_t *)malloc(jpeg_len);
  if (!new_buf)
  {
    ESP_LOGE(UVC_STREAM_TAG, "Failed to allocate memory for JPEG frame");
    return;
  }
  memcpy(new_buf, jpeg_data, jpeg_len);

  // Store in shared buffer struct
  UVCStreamHelpers::jpeg_s_fb.buf = new_buf;
  UVCStreamHelpers::jpeg_s_fb.len = jpeg_len;

  ESP_LOGI(UVC_STREAM_TAG, "Submitted JPEG frame (%zu bytes)", jpeg_len);

  // Signal that a frame is ready
  xSemaphoreGive(UVCStreamHelpers::frame_ready_sem);
}
#endif
