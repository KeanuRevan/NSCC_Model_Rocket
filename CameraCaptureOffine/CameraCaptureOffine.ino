#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"

// Camera pin definition for AI Thinker model
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define NUM_FRAMES 24000        // Number of frames to capture
#define FRAME_INTERVAL 50  // Milliseconds between frames

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-CAM MJPEG Simulation Start");

  // Camera config
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
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  // config.frame_size = FRAMESIZE_VGA;
  // config.frame_size = FRAMESIZE_SXGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  // Init camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return;
  }

  Serial.println("Camera initialized");

  // Mount SD card
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("ERROR: SD Card Mount Failed");
    return;
  }

  Serial.println("SD card mounted");

  // Capture multiple frames
  for (int i = 0; i < NUM_FRAMES; i++) {
    char filename[32];
    snprintf(filename, sizeof(filename), "/sdcard/frame_%03d.jpg", i);

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      continue;
    }

    File file = SD_MMC.open(filename, FILE_WRITE);
    if (!file) {
      Serial.printf("Failed to open %s for writing\n", filename);
    } else {
      file.write(fb->buf, fb->len);
      Serial.printf("Saved frame %d to %s (%u bytes)\n", i, filename, fb->len);
      file.close();
    }

    esp_camera_fb_return(fb);
    delay(FRAME_INTERVAL);
  }

  Serial.println("Done capturing frames.");
}

void loop() {
  // Nothing to do
}


