/*
 * ControlNode_ESP32.ino (Final: Anti-Windup, Coast-to-Stop & Fix 65535)
 */

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <SPI.h>
#include <mcp2515.h> 

#define DEBUG_MODE true // Boleh tetap true untuk pemantauan
#define ENCODER_PPR 600      // Sesuaikan jika masih salah
#define MOTOR_MAX_RPM 200    // Samakan dengan max RPM di Interface Node

#define PIN_EN  25
#define PIN_IN1 26
#define PIN_IN2 27
#define PIN_ENC_A 34
#define PIN_ENC_B 35

#define CAN_CS_PIN  5
#define CAN_INT_PIN 4

#define CAN_ID_TARGET 0x100
#define CAN_ID_PID    0x101
#define CAN_ID_ACTUAL 0x200

ESP32Encoder encoder;
MCP2515 mcp2515(CAN_CS_PIN); 

volatile int targetRPM = 0;
volatile int actualRPM = 0;
volatile int currentPWM = 0; 

volatile float Kp = 1.5, Ki = 0.3, Kd = 0.05;
float integral = 0, prevError = 0;

TaskHandle_t TaskPID;
TaskHandle_t TaskCAN;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  ledcAttach(PIN_EN, 5000, 8); 

  encoder.attachHalfQuad(PIN_ENC_A, PIN_ENC_B);
  encoder.clearCount();

  pinMode(CAN_INT_PIN, INPUT);

  SPI.begin(); 

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); 
  mcp2515.setNormalMode();

  xTaskCreatePinnedToCore(PIDLoop, "PIDTask", 4096, NULL, 2, &TaskPID, 1); 
  xTaskCreatePinnedToCore(CANLoop, "CANTask", 4096, NULL, 1, &TaskCAN, 0); 
}

void loop() {
  if (DEBUG_MODE) {
    Serial.print("Target: "); Serial.print(targetRPM);
    Serial.print(" | Actual: "); Serial.print(actualRPM);
    Serial.print(" | Error: "); Serial.print(targetRPM - actualRPM);
    Serial.print(" | PWM: "); Serial.println(currentPWM);
    delay(250); 
  } else {
    vTaskDelete(NULL); 
  }
}

// --- CORE 1: PID Control Loop ---
void PIDLoop(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(20); 
  TickType_t xLastWakeTime = xTaskGetTickCount();
  long lastCount = 0;

  for (;;) {
    long currentCount = encoder.getCount();
    long delta = currentCount - lastCount;
    lastCount = currentCount;
    actualRPM = (delta * 3000) / ENCODER_PPR; 

    if (targetRPM == 0) {
      digitalWrite(PIN_IN1, LOW);  
      digitalWrite(PIN_IN2, LOW);  
      ledcWrite(PIN_EN, 0);        
      currentPWM = 0; integral = 0; prevError = 0;
    } else {
      int safeTarget = constrain(targetRPM, -MOTOR_MAX_RPM, MOTOR_MAX_RPM);
      float error = safeTarget - actualRPM;
      
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
      currentPWM = output > 0 ? pwm : -pwm; 

      if (output > 0) {
        digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
      } else {
        digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, HIGH);
      }
      ledcWrite(PIN_EN, pwm); 
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency); 
  }
}

// --- CORE 0: CAN Bus Telemetry ---
void CANLoop(void *pvParameters) {
  TickType_t lastTransmit = xTaskGetTickCount();

  for (;;) {
    if (!digitalRead(CAN_INT_PIN)) { 
      struct can_frame rxFrame;
      
      while (mcp2515.readMessage(&rxFrame) == MCP2515::ERROR_OK) {
        // PERBAIKAN INT16_T ADA DI SINI
        if (rxFrame.can_id == CAN_ID_TARGET && rxFrame.can_dlc == 2) {
          targetRPM = (int16_t)((rxFrame.data[0] << 8) | rxFrame.data[1]);
        } 
        else if (rxFrame.can_id == CAN_ID_PID && rxFrame.can_dlc == 8) {
          int16_t rawP = (rxFrame.data[0] << 8) | rxFrame.data[1];
          int16_t rawI = (rxFrame.data[2] << 8) | rxFrame.data[3];
          int16_t rawD = (rxFrame.data[4] << 8) | rxFrame.data[5];
          Kp = rawP / 100.0;
          Ki = rawI / 100.0;
          Kd = rawD / 100.0;
        }
      }
    }

    if (xTaskGetTickCount() - lastTransmit >= pdMS_TO_TICKS(20)) {
      struct can_frame frameActual;
      frameActual.can_id = CAN_ID_ACTUAL;
      frameActual.can_dlc = 2;
      frameActual.data[0] = (actualRPM >> 8) & 0xFF;
      frameActual.data[1] = actualRPM & 0xFF;
      
      mcp2515.sendMessage(&frameActual); 
      lastTransmit = xTaskGetTickCount();
    }
    vTaskDelay(pdMS_TO_TICKS(5)); 
  }
}