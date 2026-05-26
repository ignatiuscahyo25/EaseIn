#include <Arduino.h>
#include <ESP32Encoder.h>

// Ubah ke 'true' untuk cek ultrasonik & potensio di Arduino IDE
// Ubah ke 'false' untuk menyambungkan ke Dashboard Python
const bool DEBUG_MODE = false; 

// --- KALIBRASI POTENSIO & MOTOR ---
#define MOTOR_MAX_RPM 170    
#define POT_RAW_MIN 150      
#define POT_RAW_MAX 4000     
#define ENCODER_PPR 600      

// --- DEFINISI PIN HARDWARE ---
#define PIN_EN  25
#define PIN_IN1 26 
#define PIN_IN2 27 
#define PIN_ENC_A 32
#define PIN_ENC_B 33
#define PIN_THROTTLE 34
#define PIN_TRIG 4
#define PIN_ECHO 5

// Channel PWM Khusus (Lebih stabil untuk ESP32)
const int pwmChannel = 0;
const int pwmFreq = 5000;
const int pwmResolution = 8;

ESP32Encoder encoder;

// --- VARIABEL GLOBAL ---
volatile int currentTargetRPM = 0;
volatile int actualRPM = 0;
volatile int currentPWM = 0;
volatile float Kp = 2, Ki = 1, Kd = 0;

// Filter Variabel
float smoothedPot = 0;
float smoothedTarget = 0;
const float alphaPot = 0.1;  
const float alphaRPM = 0.05; 
long lastPrintTime = 0;

TaskHandle_t TaskPID;

void setup() {
  Serial.begin(115200);

  // Setup Pin Motor menggunakan metode Channel (Kompatibel Core lama & baru)
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  ledcAttachChannel(PIN_EN, 5000, 8, 0); 
  ledcWrite(PIN_EN, 0); 

  // Setup Ultrasonik
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // Setup Encoder
  encoder.attachHalfQuad(PIN_ENC_A, PIN_ENC_B);
  encoder.clearCount();

  xTaskCreatePinnedToCore(PIDLoop, "PIDTask", 4096, NULL, 2, &TaskPID, 0);
}

long getDistance() {
  digitalWrite(PIN_TRIG, LOW); 
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); 
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duration = pulseIn(PIN_ECHO, HIGH, 15000); 
  if (duration == 0) return 999; 
  return duration * 0.034 / 2;
}

void processSerial() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    if (line.startsWith("P:")) {
      float p=0, i=0, d=0;
      sscanf(line.c_str(), "P:%f,I:%f,D:%f", &p, &i, &d);
      Kp = p; Ki = i; Kd = d;
    }
  }
}

void loop() {
  // 1. Baca Jarak Ultrasonik
  long distance = getDistance();
  int maxAllowedRPM = MOTOR_MAX_RPM; 
  
  if (distance <= 15) {
    maxAllowedRPM = 30;               // Rem Darurat
  } else if (distance > 15 && distance < 30) {
    maxAllowedRPM = MOTOR_MAX_RPM / 2; 
  }

  // 2. Baca Potensio & Filter Ganda
  int rawPot = analogRead(PIN_THROTTLE);
  smoothedPot = (alphaPot * rawPot) + ((1.0 - alphaPot) * smoothedPot);
  int filteredPot = (int)smoothedPot;

  int rawTarget = 0;
  // Deadband: Jika potensio diputar habis ke kiri (di bawah nilai min), paksakan 0
  if (filteredPot > POT_RAW_MIN) {
    int constrainedPot = constrain(filteredPot, POT_RAW_MIN, POT_RAW_MAX);
    rawTarget = map(constrainedPot, POT_RAW_MIN, POT_RAW_MAX, 0, MOTOR_MAX_RPM);
  } else {
    rawTarget = 0;
  }
  
  // Filter kedua agar pergerakan target melandai halus
  smoothedTarget = (alphaRPM * rawTarget) + ((1.0 - alphaRPM) * smoothedTarget);
  int finalTarget = (int)smoothedTarget;

  // 3. Terapkan Limit Ultrasonik
  if (finalTarget > maxAllowedRPM) {
    currentTargetRPM = maxAllowedRPM; 
  } else {
    currentTargetRPM = finalTarget;
  }

  // 4. Pengiriman Data / Debugging (Setiap 20ms)
  if (millis() - lastPrintTime >= 20) {
    if (DEBUG_MODE) {
      Serial.print("Potensio Raw: "); Serial.print(rawPot);
      Serial.print("\t| Jarak: "); Serial.print(distance);
      Serial.print(" cm\t| Target: "); Serial.print(currentTargetRPM);
      Serial.print(" RPM\t| Actual: "); Serial.println(actualRPM);
    } else {
      Serial.print(actualRPM);
      Serial.print(",");
      Serial.println(currentTargetRPM);
    }
    lastPrintTime = millis();
  }

  if (!DEBUG_MODE) processSerial();
  delay(5); 
}

void PIDLoop(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(20); 
  TickType_t xLastWakeTime = xTaskGetTickCount();
  long lastCount = 0;
  float integral = 0, prevError = 0;

  for (;;) {
    long currentCount = encoder.getCount();
    long delta = currentCount - lastCount;
    lastCount = currentCount;
    actualRPM = (delta * 3000) / ENCODER_PPR;

    if (currentTargetRPM == 0) {
      digitalWrite(PIN_IN1, LOW);  
      digitalWrite(PIN_IN2, LOW);  
      ledcWrite(PIN_EN, 0);        
      integral = 0; prevError = 0; 
    } else {
      float error = currentTargetRPM - actualRPM;
      
      float pTerm = Kp * error;
      float derivative = (error - prevError) / 0.02; 
      float dTerm = Kd * derivative;
      
      if (abs(currentPWM) < 255 || (error < 0 && currentPWM > 0) || (error > 0 && currentPWM < 0)) {
        integral += error * 0.02;
      }
      float iTerm = Ki * integral;
      
      float output = pTerm + iTerm + dTerm;
      prevError = error;

      int pwm = constrain(abs(output), 0, 255);
      
      // JIKA target ada tapi PWM terlalu kecil, suntikkan tenaga minimum (misal: 60)
      if (pwm > 0 && pwm < 60) {
        pwm = 60; 
      }
      
      currentPWM = output > 0 ? pwm : -pwm; 

      if (output > 0) {
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
      } else {
        digitalWrite(PIN_IN1, LOW); 
        digitalWrite(PIN_IN2, LOW); 
      }
      ledcWrite(PIN_EN, pwm); 
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}