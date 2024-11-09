#define WIFI_SSID "Sciaco router"
#define WIFI_PASS "12345678"
#define HOSTNAME "esp32cam"

#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/extra/esp32/wifi/sta.h>
#include <HTTPClient.h>

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
#include "esp_camera.h"

#define FRAME_SIZE FRAMESIZE_QVGA
#define WIDTH 320
#define HEIGHT 240
#define BLOCK_SIZE 4
#define W (WIDTH / BLOCK_SIZE)
#define H (HEIGHT / BLOCK_SIZE)
#define BLOCK_DIFF_THRESHOLD 1
#define IMAGE_DIFF_THRESHOLD 2
#define DEBUG 0
#define INFO 1
#define VIEWPORT_PIXELS HEIGHT / BLOCK_SIZE
#define MOTION_TIME_INTERVAL 2500

#define API_SERVER "http://192.168.117.152:5000/sensor"

using eloq::wifi;

uint16_t prev_frame[H][W] = {0};
uint16_t current_frame[H][W] = {0};
uint16_t empty_frame[H][W] = {0};
long motionView[VIEWPORT_PIXELS];

long last_motion_detection = -1;

bool capture_still() {
  camera_fb_t *frame_buffer = esp_camera_fb_get();
  if (!frame_buffer)
      return false;

  memcpy(empty_frame, current_frame, sizeof(empty_frame)); // FAST! Memcopy vs iterations so much faster.

  for (uint32_t i = 0; i < WIDTH * HEIGHT; i++)
  {
    const uint16_t x = i % WIDTH;
    const uint16_t y = floor(i / WIDTH);
    const uint8_t block_x = floor(x / BLOCK_SIZE);
    const uint8_t block_y = floor(y / BLOCK_SIZE);
    const uint8_t pixel = frame_buffer->buf[i];
    const uint16_t current = current_frame[block_y][block_x];
    current_frame[block_y][block_x] += pixel;
  }

  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      current_frame[y][x] /= BLOCK_SIZE * BLOCK_SIZE;
    }
  }

  esp_camera_fb_return(frame_buffer);
  return true;
}

bool motion_detect()
{
  uint16_t changes = 0;
  int lastBlock = 0;
  const uint16_t blocks = (WIDTH * HEIGHT) / (BLOCK_SIZE * BLOCK_SIZE);
  for (uint16_t y = 0; y < H; y++) {
    for (uint16_t x = 0; x < W; x++) {
      float current = current_frame[y][x];
      float prev = prev_frame[y][x];
      float delta = abs(current - prev) / prev;

      if (delta >= BLOCK_DIFF_THRESHOLD) {
#if DEBUG
        Serial.println(delta);
        Serial.print("diff\t");
        Serial.print(y);
        Serial.print('\t');
        Serial.println(x);
#endif

        motionView[x] = 1;
        changes++;
      }
    }
  }
  if (changes == 0) {
    return false;
  }

#if INFO
    Serial.print(":::");
#endif
    // Clear viewport to zero for next detection phase
    for (uint16_t i = 0; i < VIEWPORT_PIXELS; i++) {
#if INFO
      Serial.print(motionView[i]);
#endif
      motionView[i] = 0;
    }
#if INFO
  Serial.println(":::");
  Serial.print("Changed ");
  Serial.print(changes);
  Serial.print(" out of ");
  Serial.println(blocks);
#endif
  // return (1.0 * changes / blocks) > IMAGE_DIFF_THRESHOLD;
  return changes > IMAGE_DIFF_THRESHOLD;
}

void update_frame() {
  memcpy(prev_frame, current_frame, sizeof(prev_frame));
}

bool setup_camera(framesize_t frameSize, pixformat_t PIXEL_FORMAT) {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXEL_FORMAT;
  config.frame_size = frameSize;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  esp_camera_deinit();
  bool ok = esp_camera_init(&config) == ESP_OK;

  sensor_t *sensor = esp_camera_sensor_get();
  sensor->set_framesize(sensor, frameSize);

  return ok;
}

void notify_motion() {
  HTTPClient http;

  http.begin(API_SERVER);
  http.addHeader("Content-Type", "application/json");

  String httpRequestData = "{\"cart\":\"4e105d3d-4734-4858-addc-085fae4e0a70\"}";

  // http.GET();
  http.POST(httpRequestData);
  http.end();
}

void setup()
{
  Serial.begin(115200);
  for (uint16_t i = 0; i < VIEWPORT_PIXELS; i++) {
    motionView[i] = 0;
  }
  //    Serial.println(setup_camera(FRAME_SIZE,    PIXFORMAT_JPEG) ? "OK" : "ERR INIT");
  Serial.println(setup_camera(FRAME_SIZE, PIXFORMAT_GRAYSCALE) ? "OK" : "ERR INIT");

  while (!wifi.connect().isOk()) {
    Serial.println(wifi.exception.toString());
  }
  Serial.println("WiFi OK");

  Serial.println("End Setup...");
}

void loop()
{
    if (!capture_still()) {
#if INFO
      Serial.println("Failed capture");
#endif
      return;
    }

    if (last_motion_detection == -1) {
      last_motion_detection = millis();
    }
    if (motion_detect() && millis() - last_motion_detection > MOTION_TIME_INTERVAL) {
#if INFO
      Serial.println("Motion detected");
      notify_motion();
#endif
      last_motion_detection = millis();
    }
    update_frame();
}