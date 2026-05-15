/*
 * SingleNode_ESP32.ino (All-in-One: Sensor, PID, & PC Interface)
 * Core 1: 50Hz PID Control Loop (Prioritas Tinggi)
 * Core 0: Pembacaan Sensor (Ultrasonik & Potensio) + Komunikasi Serial PC
 */

#include <Arduino.h>
#include <ESP32Encoder.h>

// ==============================================================================
// 1. PENGATURAN MODE & SPESIFIKASI (SESUAIKAN DENGAN HARDWARE)
// ==============================================================================
#define DEBUG_MODE false     // UBAH KE 'false' JIKA INGIN DISAMBUNGKAN KE PYTHON GUI!

#define MOTOR_MAX_RPM 200    // Batas maksimal RPM fisik motor
#define ENCODER_PPR 600      // Nilai Pulses Per Revolution encoder
#define POT_RAW_MIN 1800      // Nilai raw ADC untuk batas bawah (Deadzone mati total)
#define POT_RAW_MAX 4095     // Nilai raw ADC tertinggi

// --- Hardware Pins (Terpadu) ---
#define PIN_THROTTLE 39      // (DIUBAH DARI 34) Ke pin Potensiometer
#define PIN_TRIG     32      // Ke Trigger HC-SR04
#define PIN_ECHO     33      // Ke Echo HC-SR04
#define PIN_EN       25      // Ke ENA L298N (PWM)
#define PIN_IN1      26      // Ke IN1 L298N
#define PIN_IN2      27      // Ke IN2 L298N
#define PIN_ENC_A    34      // Ke Phase A Encoder
#define PIN_ENC_B    35      // Ke Phase B Encoder

// --- Globals & Objects ---
ESP32Encoder encoder;

// Shared Variables (Volatile karena diakses menyilang antar Core)
volatile int targetRPM = 0;
volatile int actualRPM = 0;
volatile int currentPWM = 0; 

// PID Variables
volatile float Kp = 1.5, Ki = 0.3, Kd = 0.05;
float integral = 0, prevError = 0;

// Filter Variables
float smoothedTarget = 0;
const float alpha = 0.05; // Koefisien filter Low-Pass (EMA)

// Task Handles
TaskHandle_t TaskPID;
TaskHandle_t TaskSensorSerial;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); // Percepat timeout parsing serial agar tidak memblokir loop

  if (DEBUG_MODE) Serial.println("--- MEMULAI SISTEM ALL-IN-ONE ---");

  // 1. Motor Driver Setup
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  ledcAttach(PIN_EN, 5000, 8); 

  // 2. Encoder Setup
  encoder.attachHalfQuad(PIN_ENC_A, PIN_ENC_B);
  encoder.clearCount();

  // 3. Ultrasonic Setup
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // 4. Create FreeRTOS Tasks
  // Core 1 (Prioritas 2): Mengurus perhitungan PID agar selalu stabil tiap 20ms
  xTaskCreatePinnedToCore(PIDLoop, "PIDTask", 4096, NULL, 2, &TaskPID, 1); 
  
  // Core 0 (Prioritas 1): Mengurus sensor lambat dan pengiriman data Serial ke PC
  xTaskCreatePinnedToCore(SensorSerialLoop, "SensorTask", 4096, NULL, 1, &TaskSensorSerial, 0); 
}

void loop() {
  vTaskDelete(NULL); // Hapus default loop agar CPU fokus ke Task RTOS
}

// ==============================================================================
// CORE 1: PID & MOTOR CONTROL (Berjalan stabil 50Hz)
// ==============================================================================
void PIDLoop(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(20); 
  TickType_t xLastWakeTime = xTaskGetTickCount();
  long lastCount = 0;

  for (;;) {
    // 1. Hitung Actual RPM (Gunakan signed integer untuk cegah 65535 saat mundur)
    long currentCount = encoder.getCount();
    long delta = currentCount - lastCount;
    lastCount = currentCount;
    actualRPM = (int16_t)((delta * 3000) / ENCODER_PPR); 

    // 2. Coast to Stop
    if (targetRPM == 0) {
      digitalWrite(PIN_IN1, LOW);  
      digitalWrite(PIN_IN2, LOW);  
      ledcWrite(PIN_EN, 0);        
      currentPWM = 0; integral = 0; prevError = 0;
    } 
    // 3. PID Algorithm
    else {
      int safeTarget = constrain(targetRPM, -MOTOR_MAX_RPM, MOTOR_MAX_RPM);
      float error = safeTarget - actualRPM;
      
      float pTerm = Kp * error;
      float derivative = (error - prevError) / 0.02;
      float dTerm = Kd * derivative;
      
      // Anti-Windup
      if (abs(currentPWM) < 255 || (error < 0 && currentPWM > 0) || (error > 0 && currentPWM < 0)) {
        integral += error * 0.02; 
      }
      
      float iTerm = Ki * integral;
      float output = pTerm + iTerm + dTerm;
      prevError = error;

      // 4. Motor Actuation
      int pwm = constrain(abs(output), 0, 255); 
      currentPWM = output > 0 ? pwm : -pwm; 

      if (output > 0) {
        digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
      } else {
        digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, HIGH);
      }
      ledcWrite(PIN_EN, pwm); 
    }
    
    // Tunggu sisa waktu agar loop tereksekusi tepat setiap 20ms
    vTaskDelayUntil(&xLastWakeTime, xFrequency); 
  }
}

// ==============================================================================
// CORE 0: PEMBACAAN SENSOR & KOMUNIKASI PC (Berjalan stabil 50Hz)
// ==============================================================================
void SensorSerialLoop(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(20); 
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  // Variabel untuk memperlambat pengiriman data di DEBUG_MODE
  int debugCounter = 0; 

  for (;;) {
    // --- 1. PROSES PENERIMAAN SERIAL DARI PC ---
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      if (line.startsWith("P:")) {
        float p=0, i=0, d=0;
        sscanf(line.c_str(), "P:%f,I:%f,D:%f", &p, &i, &d);
        Kp = p; Ki = i; Kd = d; // Langsung update variabel volatile
      }
    }

    // --- 2. BACA & FILTER POTENSIOMETER ---
    int potValue = analogRead(PIN_THROTTLE);
    int rawTarget = 0;

    if (potValue <= POT_RAW_MIN) {
      rawTarget = 0; 
    } else {
      potValue = min(potValue, POT_RAW_MAX); 
      rawTarget = map(potValue, POT_RAW_MIN, POT_RAW_MAX, 0, MOTOR_MAX_RPM);
    }
    
    smoothedTarget = (alpha * rawTarget) + ((1.0 - alpha) * smoothedTarget);
    int finalTarget = (int)smoothedTarget; 

    // --- 3. BACA ULTRASONIK & HITUNG LIMIT ---
    digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    
    // Timeout dipersingkat (15000us = ~2.5 meter) agar tidak memblokir loop terlalu lama
    long duration = pulseIn(PIN_ECHO, HIGH, 15000); 
    long distance = (duration == 0) ? 999 : (duration * 0.034 / 2);

    int maxAllowedRPM = MOTOR_MAX_RPM; 
    String statusKeamanan = "";
    
    if (distance <= 15) {
      maxAllowedRPM = 0; 
      statusKeamanan = "CRITICAL";
    } else if (distance > 15 && distance < 50) {
      maxAllowedRPM = MOTOR_MAX_RPM / 2; 
      statusKeamanan = "WARNING";
    } else {
      maxAllowedRPM = MOTOR_MAX_RPM; 
      statusKeamanan = "SAFE";
    }

    // Terapkan limit sensor ke target akhir
    targetRPM = (finalTarget > maxAllowedRPM) ? maxAllowedRPM : finalTarget;

    // --- 4. PENGIRIMAN SERIAL (TELEMETRI) ---
    if (DEBUG_MODE) {
      debugCounter++;
      if (debugCounter >= 12) { // Print setiap ~240ms (agar bisa dibaca mata)
        Serial.print("Jarak: "); Serial.print(distance); Serial.print(" cm ("); Serial.print(statusKeamanan); Serial.print(") | ");
        Serial.print("Target Akhir: "); Serial.print(targetRPM); Serial.print(" | ");
        Serial.print("Actual RPM: "); Serial.print(actualRPM); Serial.print(" | ");
        Serial.print("PWM: "); Serial.println(currentPWM);
        debugCounter = 0;
      }
    } else {
      // Format untuk Python GUI (dikirim terus setiap 20ms / 50Hz)
      Serial.print(actualRPM);
      Serial.print(",");
      Serial.println(targetRPM);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency); 
  }
}