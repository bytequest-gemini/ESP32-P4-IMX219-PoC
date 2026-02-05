# ESP32-P4-IMX219-PoC
Experimental Proof of Concept enabling Sony IMX219 (RPi Camera v2) color streaming on ESP32-P4 via MIPI CSI-2 and software demosaicing (V4L2).
# ESP32-P4 Sony IMX219 Color Stream (PoC)

![Status](https://img.shields.io/badge/Status-Experimental-orange)
![Chip](https://img.shields.io/badge/Chip-ESP32--P4-red)
![Camera](https://img.shields.io/badge/Sensor-Sony_IMX219-blue)

This project is a **Proof of Concept (PoC)** demonstrating how to interface a **Sony IMX219 (Raspberry Pi Camera v2)** with the **ESP32-P4** using the MIPI CSI-2 interface to get a color video stream.

Since the official ISP (Image Signal Processor) driver for the ESP32-P4 is not yet fully available for public use, this project implements a **hybrid software/hardware pipeline** to bypass the limitation and achieve a color image.

### 🚀 Key Features

* **V4L2 Implementation:** Uses the standard `video4linux2` API via `esp_video` component.
* **RAW10 Capture:** Captures raw Bayer data (SBGGR10) directly from the MIPI sensor.
* **Software Demosaicing:** Implements a custom CPU-based bilinear interpolation (`demosaic_bggr_to_rgb`) to convert RAW10 to RGB888.
* **Hardware Encoding:** Feeds the RGB data into the ESP32-P4's internal **JPEG Encoder** for compression.
* **HTTP Stream:** Serves a live MJPEG stream over Wi-Fi.

### ⚡ Performance

* **Input Resolution:** 1920x1080 (RAW10)
* **Output Resolution:** 960x540 (Downscaled during demosaicing for performance)
* **Framerate:** ~8 FPS (Bottlenecked by single-core software demosaicing)
* **Format:** Color MJPEG

### 🔌 Pinout & Wiring

This configuration uses the standard **15-pin FPC MIPI** connector layout found on most ESP32-P4 development boards (e.g., Espressif P4-Function-EV-Board, Waveshare, etc.).

| Pin Function | ESP32-P4 GPIO | Description |
| :--- | :--- | :--- |
| **I2C SDA** | `GPIO 7` | Control Data |
| **I2C SCL** | `GPIO 8` | Control Clock |
| **XCLK** | `GPIO 20` | 24MHz Master Clock (Generated via LEDC) |
| **MIPI Lanes** | *Internal* | Uses internal MIPI PHY (2-lane configuration) |

### 🛠️ Dependencies

* **ESP-IDF:** v5.3 (Master branch recommended for P4 support)
* **Required Components:**
    * `esp_video`
    * `esp_cam_sensor`
    * `driver` (LEDC for XCLK)

### 🏗️ Build & Flash

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/YOUR_USERNAME/ESP32-P4-IMX219-PoC.git](https://github.com/YOUR_USERNAME/ESP32-P4-IMX219-PoC.git)
    cd ESP32-P4-IMX219-PoC
    ```

2.  **Configure Wi-Fi:**
    Open `main/main.c` and edit the SSID/PASSWORD definitions (or use menuconfig if implemented).

3.  **Build and Flash:**
    ```bash
    idf.py set-target esp32p4
    idf.py build flash monitor
    ```

4.  **View Stream:**
    Wait for the log `Got IP: ...`, then open your browser and navigate to:
    `http://<ESP32-IP-ADDRESS>/`

### ⚠️ Technical Notes

* **Demosaicing:** The conversion from Bayer to RGB is currently done on the CPU. This is computationally expensive and limits the framerate. Once Espressif releases the full ISP driver, this step can be offloaded to hardware for 30FPS+ performance.
* **Image Quality:** Since there is no hardware ISP processing (Auto White Balance, Auto Exposure, Gamma Correction), the image relies on the raw sensor output and might appear dark or green-tinted depending on lighting conditions.

### 📜 License

MIT License.

### 🤝 Acknowledgments

* **Hardware & Testing:** Developed and tested by **[bytequest-gemini]**.
* **AI Assistance:** Special thanks to **Google Gemini** for providing technical guidance on the V4L2 implementation, reverse-engineering the register map, and optimizing the software demosaicing algorithm.
* **Community:** Thanks to the ESP32 community for the continuous research on P4 capabilities.
