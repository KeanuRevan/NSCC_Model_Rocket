# ESP32-CAM Setup and Image Capture Guide

This guide explains how to set up an ESP32-CAM module, upload code using an Arduino Uno, and use the module to capture images to an SD card.

---

## 🧰 What You Need

* ESP32-CAM module (AI Thinker model)
* Arduino Uno (as USB-to-Serial adapter)
* Jumper wires
* MicroSD card (formatted as FAT32)
* Arduino IDE (latest version)

---

## 🔌 Wiring: Arduino Uno to ESP32-CAM

| ESP32-CAM Pin | Arduino Uno Pin    |
| ------------- | ------------------ |
| U0R (RX)      | RX (pin 0)         |
| U0T (TX)      | TX (pin 1)         |
| GND           | GND                |
| 5V            | 5V                 |
| GPIO0         | GND *(for upload)* |

![image](https://github.com/user-attachments/assets/2659a3e1-04f2-4000-a4ac-4aabd68e0b6b)

> ⚠️ **Important:** GPIO0 must be connected to GND to enter programming mode. Disconnect it from GND after uploading.

---

## ⚙️ Arduino IDE Configuration (Tools Menu)

Ensure the following settings under the **Tools** menu:

* **Board**: `AI Thinker ESP32-CAM` *(or `ESP32 Dev Module` if AI Thinker is unavailable, but set camera pins manually)*
* **Port**: COM port of the Arduino Uno
* **Upload Speed**: `115200`
* **Flash Frequency**: `40MHz`
* **Flash Mode**: `QIO`
* **Partition Scheme**: `Huge APP (3MB No OTA/1MB SPIFFS)`
* **PSRAM**: `Enabled`
* **Programmer**: `AVRISP mkII`

---

## 🚀 Uploading Code Using Arduino Uno

1. **Wire your devices** as shown in the table above.
2. In Arduino IDE, configure the **Tools** settings as shown above.
3. Press **RESET** on the ESP32-CAM just after hitting **Upload**.
4. After successful upload, **disconnect GPIO0 from GND** and press **RESET** again to run the program.

---

## 💾 What the Code Does

This sketch initializes the ESP32-CAM camera and saves a series of images to the SD card in JPEG format.

### Key Features:

* Captures **30,000 frames** (`NUM_FRAMES`) at 100ms intervals.
* Stores images as `frame_000.jpg`, `frame_001.jpg`, ... on the SD card.

### Code Explanation:

```cpp
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
```

Includes necessary libraries for camera, SD card, and ESP32 internals.

```cpp
#define NUM_FRAMES 30000
#define FRAME_INTERVAL 100
```

Captures 30,000 images with a 100ms delay between each.

```cpp
camera_config_t config;
// Sets all the camera pin numbers, resolution, format, and quality
```

Configures the camera's pin mapping and frame settings.

```cpp
esp_camera_init(&config);
```

Initializes the ESP32-CAM with the given config.

```cpp
SD_MMC.begin("/sdcard", true);
```

Mounts the microSD card.

```cpp
camera_fb_t *fb = esp_camera_fb_get();
```

Takes a snapshot and returns a frame buffer.

```cpp
file.write(fb->buf, fb->len);
```

Writes image data to file on SD card.

```cpp
esp_camera_fb_return(fb);
```

Releases the frame buffer for reuse.

---

## ✅ Tips for Success

* Use a good quality **power supply** (at least 5V 1A).
* Make sure the **microSD card is formatted as FAT32**.
* If uploads fail, try pressing RESET just before or after upload begins.
* Avoid opening Serial Monitor during uploads to prevent interference.

---

## 📸 Result

After successful execution, your SD card will contain JPEG images named like `frame_000.jpg`, `frame_001.jpg`, etc. These can be downloaded and viewed on a PC or processed further.

---

## 📚 Resources

* [CircuitSchools Guide](https://www.circuitschools.com/how-to-program-upload-the-code-to-esp32-cam-using-arduino-or-programmer/)
* [ESP32-CAM AI Thinker Pinout](https://randomnerdtutorials.com/esp32-cam-ai-thinker-pinout/)
* [Arduino IDE ESP32 Setup](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)

---

Feel free to adapt this code and setup to build motion detectors, timelapse projects, wildlife cams, or other creative ESP32-CAM projects!
