#include <Arduino.h>

// ---------------------------------------------------------------
// HARDWARE PIN MAPPING (ESP32-S3)
// Indexing: [0]=RF, [1]=RM(Offline), [2]=RR, [3]=LF, [4]=LM(Offline), [5]=LR
// ---------------------------------------------------------------
const int PWM_PINS[6] = {41, 5,  7,  6,  42, 4};
const int DIR_PINS[6] = {40, 10, 12, 11, 13, 9};
const int ENC_PINS[6] = {39, 16, 18, 17, 8,  15};

const char* MOTOR_NAMES[6] = {"RF", "RM(Off)", "RR", "LF", "LM(Off)", "LR"};

// PER-MOTOR SPEED TABLE (Recalibrated)
//                         FWD   BWD   RIGHT  LEFT
int motorSpeeds[6][4] = {
  {86,   100,  164,  136},   // M1 (RF)
  {90,   90,   150,  150},   // M2 (RM) - Disconnected
  {87,   118,  178,  136},   // M3 (RR)
  {90,   93,   154,  162},   // M4 (LF)
  {90,   90,   150,  150},   // M5 (LM) - Disconnected
  {96,   81,   158,  136},   // M6 (LR)
};
// CMD index mapping: W=0, S=1, D=2, A=3

// ---------------------------------------------------------------
// ENCODER & RPS VARIABLES (8 Magnets = 8 Ticks per Revolution)
// ---------------------------------------------------------------
const unsigned long debounceDelay = 15; // ms

volatile unsigned long tickInterval[6] = {0, 0, 0, 0, 0, 0};
volatile unsigned long lastInterruptTime[6] = {0, 0, 0, 0, 0, 0};
volatile bool newTick[6] = {false, false, false, false, false, false};

float currentRPS[6]  = {0, 0, 0, 0, 0, 0};
float smoothedRPS[6] = {0, 0, 0, 0, 0, 0};

char activeCmd = 'X';
int speedOffset = 0;
unsigned long lastPrintTime = 0;

// ---------------------------------------------------------------
// INTERRUPT SERVICE ROUTINES (ISRs)
// ---------------------------------------------------------------
void IRAM_ATTR calcRev(int idx) {
  unsigned long currentTime = millis();
  unsigned long interval = currentTime - lastInterruptTime[idx];
  if (interval > debounceDelay) {
    tickInterval[idx] = interval;
    lastInterruptTime[idx] = currentTime;
    newTick[idx] = true;
  }
}

void IRAM_ATTR isr0() { calcRev(0); }
void IRAM_ATTR isr1() { calcRev(1); }
void IRAM_ATTR isr2() { calcRev(2); }
void IRAM_ATTR isr3() { calcRev(3); }
void IRAM_ATTR isr4() { calcRev(4); }
void IRAM_ATTR isr5() { calcRev(5); }

// ---------------------------------------------------------------
// MOTOR MOVEMENT LOGIC
// ---------------------------------------------------------------
void applyMovement(char cmd) {
  if (cmd == 'X') {
    for (int i = 0; i < 6; i++) analogWrite(PWM_PINS[i], 0);
    return;
  }

  int cmdIdx;
  bool rightDir, leftDir;

  if      (cmd == 'W') { cmdIdx = 0; rightDir = HIGH; leftDir = HIGH; }
  else if (cmd == 'S') { cmdIdx = 1; rightDir = LOW;  leftDir = LOW;  }
  else if (cmd == 'D') { cmdIdx = 2; rightDir = LOW;  leftDir = HIGH; }
  else if (cmd == 'A') { cmdIdx = 3; rightDir = HIGH; leftDir = LOW;  }
  else return;

  // Right Side Motors (0, 1, 2)
  for (int i = 0; i < 3; i++) {
    digitalWrite(DIR_PINS[i], rightDir);
    int basePwm = motorSpeeds[i][cmdIdx];
    int finalPwm = (basePwm == 0) ? 0 : constrain(basePwm + speedOffset, 0, 255);
    analogWrite(PWM_PINS[i], finalPwm);
  }

  // Left Side Motors (3, 4, 5)
  for (int i = 3; i < 6; i++) {
    digitalWrite(DIR_PINS[i], leftDir);
    int basePwm = motorSpeeds[i][cmdIdx];
    int finalPwm = (basePwm == 0) ? 0 : constrain(basePwm + speedOffset, 0, 255);
    analogWrite(PWM_PINS[i], finalPwm);
  }
}

void handleCommand(char cmd) {
  cmd = toupper(cmd);

  if (cmd == 'P') {
    speedOffset += 10;
    Serial.print("[CMD] Speed Offset Increased: ");
    Serial.println(speedOffset);
    if (activeCmd != 'X') applyMovement(activeCmd);
  }
  else if (cmd == 'M') {
    speedOffset -= 10;
    Serial.print("[CMD] Speed Offset Decreased: ");
    Serial.println(speedOffset);
    if (activeCmd != 'X') applyMovement(activeCmd);
  }
  else if (cmd == 'W' || cmd == 'A' || cmd == 'S' || cmd == 'D' || cmd == 'X') {
    activeCmd = cmd;
    if (activeCmd == 'X') speedOffset = 0;
    Serial.print("[CMD] Active Mode: ");
    Serial.println(activeCmd);
    applyMovement(activeCmd);
  }
}

// ---------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  for (int i = 0; i < 6; i++) {
    pinMode(PWM_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(ENC_PINS[i], INPUT_PULLUP);
    digitalWrite(DIR_PINS[i], LOW);
    analogWrite(PWM_PINS[i], 0);
  }

  attachInterrupt(digitalPinToInterrupt(ENC_PINS[0]), isr0, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[1]), isr1, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[2]), isr2, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[3]), isr3, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[4]), isr4, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[5]), isr5, FALLING);

  Serial.println("====================================================");
  Serial.println("ESP32-S3 Rover Motor & RPS Test Ready!");
  Serial.println("Commands: W=Forward, S=Backward, A=Left, D=Right, X=Stop");
  Serial.println("Speed Adjust: P=(+10 PWM), M=(-10 PWM)");
  Serial.println("====================================================");
}

// ---------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------
void loop() {
  unsigned long currentMillis = millis();

  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    if (incomingChar != '\n' && incomingChar != '\r') {
      handleCommand(incomingChar);
    }
  }

  for (int i = 0; i < 6; i++) {
    if (newTick[i]) {
      noInterrupts();
      unsigned long interval = tickInterval[i];
      newTick[i] = false;
      interrupts();

      if (interval > 0) {
        // 1 tick = 1/8 rev -> RPS = 1000 / (8 * interval) = 125.0 / interval
        currentRPS[i] = 125.0 / (float)interval;
        if (smoothedRPS[i] == 0.0) smoothedRPS[i] = currentRPS[i];
        smoothedRPS[i] = (smoothedRPS[i] * 0.7) + (currentRPS[i] * 0.3);
      }
    }

    if (currentMillis - lastInterruptTime[i] > 1000) {
      if (smoothedRPS[i] > 0) {
        currentRPS[i] = 0;
        smoothedRPS[i] = 0;
      }
    }
  }

  if (currentMillis - lastPrintTime >= 300) {
    Serial.print("Mode: ");
    Serial.print(activeCmd);
    Serial.print(" | Offset: ");
    Serial.print(speedOffset);
    Serial.print(" || RPS -> ");

    for (int i = 0; i < 6; i++) {
      // if (i == 1 || i == 4) continue;
      Serial.print(MOTOR_NAMES[i]);
      Serial.print(": ");
      Serial.print(smoothedRPS[i], 2);
      if (i < 5) Serial.print(" | ");
    }
    Serial.println();
    lastPrintTime = currentMillis;
  }
}