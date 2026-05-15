# DC Motor PID Control & Telemetry Dashboard

Proyek ini adalah sistem pengendalian motor DC berbasis PID dengan fitur pemantauan (*telemetry*) waktu-nyata. Proyek ini mendukung dua jenis arsitektur perangkat keras: **Sistem Terdistribusi** (menggunakan komunikasi CAN Bus antar dua ESP32) dan **Sistem Terpadu** (menggunakan satu ESP32).

## 🛠️ Requirements (Kebutuhan Sistem)

### Kebutuhan Perangkat Keras (Hardware)
* Mikrokontroler ESP32 (1 atau 2 buah, tergantung arsitektur)
* Motor Driver L298N
* Motor DC dengan *Rotary Encoder* (Quadrature)
* Sensor Jarak Ultrasonik HC-SR04
* Potensiometer (Analog Throttle)
* Modul CAN Bus MCP2515 (Hanya untuk arsitektur terdistribusi)

### Kebutuhan Arduino IDE (Libraries)
Instal *library* berikut melalui **Library Manager** di Arduino IDE:
* **`ESP32Encoder`** (oleh Kevin Harrington) - Untuk membaca pulsa *quadrature encoder* secara presisi tanpa membebani CPU.
* **`mcp2515`** (oleh autowp) - Untuk komunikasi protokol CAN Bus menggunakan antarmuka SPI.

*Catatan: Pastikan Anda menggunakan paket ESP32 Board Manager versi 3.x ke atas untuk dukungan API PWM `ledcAttach`.*

### Kebutuhan Python (Dependencies)
Aplikasi antarmuka pemantauan berjalan menggunakan Python 3. Buka terminal atau Command Prompt dan jalankan perintah berikut untuk menginstal semua pustaka yang dibutuhkan:

```bash
pip install PyQt6 pyqtgraph pyserial

```

---

## 📂 Daftar Kode & Penjelasan Arsitektur

### Arsitektur 1: Sistem Terdistribusi (CAN Bus)

Menggunakan dua buah ESP32 yang saling berkomunikasi secara *real-time* via modul CAN Bus pada kecepatan 500 KBPS. Sangat cocok untuk mensimulasikan sistem telegraf dan telemetri otomotif modern.

* **1. `InterfaceNode_ESP32.ino` (Node Interaksi PC & Sensor)**
Berfungsi sebagai jembatan eksternal. Node ini membaca input analog dari **Potensiometer** (menggunakan *Low-Pass Filter* untuk meredam osilasi) sebagai penentu kecepatan, serta membaca sensor **Ultrasonik** yang bertindak sebagai *Speed Limiter* otomatis jika ada rintangan (*Collision Avoidance*). Node ini menerjemahkan data tersebut ke dalam paket CAN Bus, sekaligus mengirim data Serial kecepatan tinggi ke PC.
* **2. `ControlNode_ESP32.ino` (Node Aktuator PID)**
Berfungsi sebagai eksekutor tingkat rendah. Node ini menerima instruksi target RPM via CAN Bus, membaca kecepatan putar aktual dari **Encoder**, dan mengeksekusi kendali **PID** pada kecepatan 50Hz. Node ini memiliki algoritma *Anti-Windup* dan *Coast-to-Stop* untuk respons pengereman yang realistis, lalu mengirimkan kembali data kecepatan aktual ke Interface Node.

### Arsitektur 2: Sistem Terpadu (All-in-One)

Solusi alternatif yang menggabungkan seluruh fungsi ke dalam satu papan ESP32 tanpa perantara CAN Bus, memanfaatkan arsitektur *Dual-Core* bawaan chip (FreeRTOS) untuk mencegah *lag* pembacaan sensor.

* **3. `SingleNode_ESP32.ino**`
Kode gabungan (*all-in-one*).
* **Core 1 (Prioritas Tinggi):** Didedikasikan khusus menjalankan `PIDLoop` agar siklus pembacaan encoder dan pengiriman sinyal aktuasi ke L298N tetap stabil di 20ms (50Hz).
* **Core 0 (Prioritas Menengah):** Didedikasikan menjalankan `SensorSerialLoop` untuk memproses sensor eksternal (potensiometer & ultrasonik) serta komunikasi dengan aplikasi *dashboard* di PC.



### Aplikasi Pemantauan PC

* **4. `MotorDashboard.py` (Python GUI)**
Aplikasi *dashboard* berbasis **PyQt6** dan **PyQtGraph** untuk pemantauan sistem. Fitur utamanya:
* **Live Plotting 50Hz:** Visualisasi grafik kecepatan aktual (Cyan) terhadap target (Amber) secara *real-time* tanpa membuat antarmuka membeku (menggunakan `QThread` khusus).
* **Live Tuning:** Panel PID yang memungkinkan pengguna mengubah konstanta $K_p$, $K_i$, dan $K_d$ secara langsung (*on-the-fly*).
* **Auto-Connect & Status Bar:** Pendeteksi port COM ESP32 otomatis yang dilengkapi *live metrics* error dan Hz rate.
* **CSV Logger:** Pengeksporan riwayat putaran motor dan waktu ke format `.csv` untuk keperluan analisis jurnal/laporan.



---

## Cara Menjalankan (*Quick Start*)

1. **Pilih Arsitektur:** Rangkai perangkat keras Anda (apakah menggunakan mode CAN Bus dua node, atau mode tunggal).
2. **Setup Mode ESP32:** Buka kode `.ino` yang terhubung langsung ke PC via kabel USB (`InterfaceNode` atau `SingleNode`). Pastikan variabel `#define DEBUG_MODE false` agar ESP32 mengirim format data khusus untuk aplikasi GUI.
3. **Upload Kode:** *Compile* dan *Upload* program menggunakan Arduino IDE. Tutup *Serial Monitor* bawaan Arduino jika sudah selesai.
4. **Jalankan Dashboard:** Buka terminal/PowerShell PC Anda, arahkan ke folder yang sama, lalu ketik `python MotorDashboard.py`.
5. **Mulai Pengujian:** Pilih port COM yang terdeteksi, klik **Connect**, dan cobalah memutar potensiometer atau mengubah jarak di depan sensor ultrasonik. Grafik akan langsung beraksi!

```

```
