/*
 * InterfaceNode_ESP32.ino (Revisi: Deadzone Potensio & Sensor Limiter)
 */

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h> 

#define DEBUG_MODE true // Ubah ke false saat dihubungkan ke GUI Python

// --- KALIBRASI POTENSIO & MOTOR ---
#define MOTOR_MAX_RPM 200 // Batas maksimal RPM motor fisik Anda
// UBAH POT_RAW_MIN sedikit di atas nilai terendah potensio Anda. 
// Jika terendah 160, set ke 250 agar pasti mati (NOL) saat mentok kiri.
#define POT_RAW_MIN 1800  
#define POT_RAW_MAX 3200 

// --- VARIABEL LOW-PASS FILTER (EMA) ---
float smoothedTarget = 0;
const float alpha = 0.05; 

// --- Hardware Pins ---
#define PIN_THROTTLE 34 
#define PIN_TRIG     32 
#define PIN_ECHO     33 

#define CAN_CS_PIN   5  
#define CAN_INT_PIN  4  

#define CAN_ID_TARGET 0x100
#define CAN_ID_PID    0x101
#define CAN_ID_ACTUAL 0x200

MCP2515 mcp2515(CAN_CS_PIN); 

long lastPrintTime = 0;
long lastCanTxTime = 0;
int currentActualRPM = 0;
int currentTargetRPM = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(CAN_INT_PIN, INPUT);

  if (DEBUG_MODE) Serial.println("--- MEMULAI SISTEM (DEADZONE & LIMITER) ---");

  SPI.begin(); 
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); 
  mcp2515.setNormalMode();
}

long getDistance() {
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, 30000); 
  if (duration == 0) return 999; 
  return duration * 0.034 / 2; 
}

void processSerial() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    if (line.startsWith("P:")) {
      float p=0, i=0, d=0;
      sscanf(line.c_str(), "P:%f,I:%f,D:%f", &p, &i, &d);
      
      int16_t scaleP = p * 100;
      int16_t scaleI = i * 100;
      int16_t scaleD = d * 100;
      
      struct can_frame framePID;
      framePID.can_id = CAN_ID_PID;
      framePID.can_dlc = 8;
      framePID.data[0] = (scaleP >> 8) & 0xFF;
      framePID.data[1] = scaleP & 0xFF;
      framePID.data[2] = (scaleI >> 8) & 0xFF;
      framePID.data[3] = scaleI & 0xFF;
      framePID.data[4] = (scaleD >> 8) & 0xFF;
      framePID.data[5] = scaleD & 0xFF;
      framePID.data[6] = 0;
      framePID.data[7] = 0;
      
      mcp2515.sendMessage(&framePID);
    }
  }
}

void loop() {
  // 1. Baca ADC Mentah Potensio
  int potValue = analogRead(PIN_THROTTLE);
  int rawTarget = 0;

  // 2. DEADZONE LOGIC (Paksa Berhenti ke Nol)
  if (potValue <= POT_RAW_MIN) {
    rawTarget = 0; // Jika mentok kiri, target RPM MUTLAK = 0
  } else {
    // Jika lebih besar dari batas bawah, map secara proporsional
    potValue = min(potValue, POT_RAW_MAX); // Pastikan tidak lewat 4095
    rawTarget = map(potValue, POT_RAW_MIN, POT_RAW_MAX, 0, MOTOR_MAX_RPM);
  }

  // 3. Terapkan Low-Pass Filter (Biar Halus & Tidak Loncat-Loncat)
  smoothedTarget = (alpha * rawTarget) + ((1.0 - alpha) * smoothedTarget);
  int finalTarget = (int)smoothedTarget; 

  // 4. SAFETY LIMITER LOGIC (Ultrasonik menentukan Batas Kecepatan Maksimal)
  long distance = getDistance();
  int maxAllowedRPM = MOTOR_MAX_RPM; // Default batas atas = 200 RPM
  String statusKeamanan = "";
  
  if (distance <= 15) {
    maxAllowedRPM = 20; // Harus berhenti darurat
    statusKeamanan = "CRITICAL (REM DARURAT)";
  } else if (distance > 15 && distance < 30) {
    maxAllowedRPM = MOTOR_MAX_RPM / 2; // Kecepatan tidak boleh lewat 50% (100 RPM)
    statusKeamanan = "WARNING (LIMIT 50%)";
  } else {
    maxAllowedRPM = MOTOR_MAX_RPM; // Bebas gas sampai mentok 100% (200 RPM)
    statusKeamanan = "SAFE (NORMAL)";
  }

  // 5. PENENTUAN TARGET AKHIR
  // Target yang digunakan adalah permintaan potensio, TETAPI tidak boleh melebihi batas maksimal sensor!
  if (finalTarget > maxAllowedRPM) {
    currentTargetRPM = maxAllowedRPM; // Dipangkas (clamping) oleh sensor
  } else {
    currentTargetRPM = finalTarget;   // Ikuti gas potensio karena masih aman
  }

  // 6. Baca pesan CAN masuk
  if (!digitalRead(CAN_INT_PIN)) { 
    struct can_frame rxFrame;
    while (mcp2515.readMessage(&rxFrame) == MCP2515::ERROR_OK) {
      if (rxFrame.can_id == CAN_ID_ACTUAL && rxFrame.can_dlc == 2) {
        currentActualRPM = (rxFrame.data[0] << 8) | rxFrame.data[1];
      }
    }
  }

  // 7. Transmit Target ke Control Node via CAN
  if (millis() - lastCanTxTime >= 20) {
    struct can_frame frameTarget;
    frameTarget.can_id = CAN_ID_TARGET;
    frameTarget.can_dlc = 2;
    frameTarget.data[0] = (currentTargetRPM >> 8) & 0xFF;
    frameTarget.data[1] = currentTargetRPM & 0xFF;
    
    mcp2515.sendMessage(&frameTarget);
    lastCanTxTime = millis();
  }

  // 8. Kirim data ke Serial
  if (DEBUG_MODE) {
    if (millis() - lastPrintTime >= 250) {
      Serial.print("Potensio Raw: "); Serial.print(potValue); Serial.print(" | ");
      Serial.print("Jarak: "); Serial.print(distance); Serial.print(" cm | ");
      Serial.print("Target Akhir: "); Serial.print(currentTargetRPM); Serial.print(" | ");
      Serial.print("Actual: "); Serial.println(currentActualRPM);
      lastPrintTime = millis();
    }
  } else {
    if (millis() - lastPrintTime >= 20) {
      Serial.print(currentActualRPM);
      Serial.print(",");
      Serial.println(currentTargetRPM);
      lastPrintTime = millis();
    }
  }

  processSerial();
  delay(5); 
}