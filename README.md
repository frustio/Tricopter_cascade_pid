# 🚁 Tricopter Cascade PID Flight Controller

Flight Controller (FC) kustom berbasis mikrokontroler STM32F407 untuk Tricopter, ditulis menggunakan Arduino IDE. Sistem ini menggunakan arsitektur **Cascade PID** untuk menghasilkan stabilitas terbang yang halus pada sumbu Roll, Pitch, dan Yaw, dilengkapi dengan fitur vectoring servo pada 2 motor depan.

## ✨ Fitur Utama
* **Cascade PID Control:** Sistem kendali bertingkat untuk respon aktuator yang lebih presisi.
* **Protokol ELRS (CRSF):** Komunikasi *low-latency* menggunakan ExpressLRS.
* **Sensor Fusi 9-Axis:** Menggabungkan data MPU6050 (Gyro/Accel) dan HMC5883L (Magnetometer) untuk pembacaan *heading* dan orientasi absolut.
* **Servo Vectoring:** Kendali servo presisi untuk mekanisme *yaw* pada tricopter.

## 🛠️ Kebutuhan Hardware
* **Microcontroller:** STM32F407
* **IMU:** MPU6050
* **Kompas:** HMC5883L
* **Receiver:** ExpressLRS (ELRS) Nano Receiver
* **Aktuator:** 3x ESC (Motor BLDC) & 2x Motor Servo 

## 🔌 Skema Pin (Wiring / Pinout)
Pastikan koneksi kabel sesuai dengan konfigurasi berikut:

| Komponen | Pin Board | Keterangan |
| :--- | :--- | :--- |
| **MPU6050 (I2C)** | SDA: `PB9`, SCL: `PB8` | Pull-up resistor mungkin diperlukan |
| **MPU6050 (INT)** | `PB12` | Interrupt pin |
| **HMC5883L (I2C)**| SDA: `PB9`, SCL: `PB8` | Paralel dengan MPU6050 |
| **ELRS Rx (CRSF)**| TX: `PD6`, RX: `PD5` | Menggunakan `Serial2` |
| **Telemetri (Opt)** | TX: `PA1`, RX: `PA0` | Menggunakan `Serial1` (57600 baud) |
| **ESC / Motor** | M1: `PE0`, M2: `PE1`, M3: `PE2` | PWM Output |
| **Servo Yaw** | S1: `PE3`, S2: `PE4` | PWM Output |

## 📚 Kebutuhan Library (Dependencies)
Sebelum melakukan *compile*, pastikan library berikut sudah terinstal di Arduino IDE:
1. `MPU6050_6Axis_MotionApps20.h` (oleh Jeff Rowberg)
2. `HMC5883L.h` (oleh Korneliusz Jarzebski atau sejenisnya)
3. `CRSFforArduino.hpp` (untuk komunikasi ELRS)
4. `Servo.h` (bawaan Arduino)

## ⚙️ Catatan Tuning PID
Variabel PID terletak di bagian atas kode. Karena menggunakan *Cascade PID*:
* **Loop Luar (Angle):** Mengatur target sudut kemiringan.
* **Loop Dalam (Rate):** Mengatur kecepatan sudut (Gyro).

## 👨‍💻 Dibuat Oleh
**@frustio** - 2025
