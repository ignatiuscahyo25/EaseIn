# All-in-One ESP32 DC Motor PID Control & Telemetry Dashboard

Sistem terpadu untuk pengendalian motor DC berbasis PID dengan fitur pemantauan (*telemetry*) waktu-nyata. Proyek ini mengintegrasikan pembacaan sensor, algoritma kendali PID dengan *anti-windup*, dan komunikasi serial ke dalam satu mikrokontroler ESP32 menggunakan arsitektur FreeRTOS (*Dual-Core*), yang divisualisasikan melalui *dashboard* antarmuka GUI Python.

Sistem ini sangat cocok digunakan untuk eksperimen laboratorium sistem kontrol, pengujian aktuator mekanis, maupun pengembangan robotika dasar.

## 🛠️ Requirements (Kebutuhan Sistem)

### Kebutuhan Perangkat Keras (Hardware)
* Mikrokontroler ESP32
* Motor Driver L298N
* Motor DC dengan *Rotary Encoder* (Quadrature)
* Sensor Jarak Ultrasonik HC-SR04
* Potensiometer (Analog Throttle)

### Kebutuhan Arduino IDE (Libraries)
Instal *library* berikut melalui **Library Manager** di Arduino IDE:
* **`ESP32Encoder`** (oleh Kevin Harrington) - Untuk membaca pulsa *quadrature encoder* secara presisi tanpa membebani CPU.

*Catatan: Pastikan Anda menggunakan ESP32 Board Manager versi 3.x ke atas, karena kode ini memanfaatkan API PWM `ledcAttach` terbaru.*

### Kebutuhan Python (Dependencies)
Dashboard berjalan menggunakan Python 3. Buka terminal/Command Prompt dan jalankan perintah berikut untuk menginstal semua *dependency* yang dibutuhkan:
```bash
pip install PyQt6 pyqtgraph pyserial
