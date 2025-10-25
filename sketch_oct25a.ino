#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>
#include "BluetoothSerial.h"

// ==================== BACKEND DEFINITIONS ====================
BluetoothSerial SerialBT;

// --- Pin Definitions ---
#define STEP_PIN 26
#define DIR_PIN 25
#define PUMP_PIN 33
#define HEATER_PIN 32
#define ONE_WIRE_BUS 27

// --- DS18B20 Temperature Setup ---
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// --- Control Variables ---
bool stopFlag = false;
bool pumpRunning = false;
int pumpSpeed = 180; // PWM 0–255
const int PULSES_PER_REV = 6400;
int stepsPerSec = 1500;
int oscAngleDeg = 40;
long stepsToMove = (PULSES_PER_REV * oscAngleDeg) / 360;

enum OscState { TO_POS, TO_ZERO_FROM_POS, TO_NEG, TO_ZERO_FROM_NEG };
OscState oscState = TO_POS;
long currentStepPos = 0;
unsigned long lastStepTime = 0;
bool currentDir = HIGH;

// Bluetooth buffer
String btBuffer = "";

// Temperature + heater tracking
float currentTemp = 0.0;
bool heaterState = false;

// ==================== FRONTEND (TFT + TOUCH) ====================
TFT_eSPI tft = TFT_eSPI();
#define TS_CS 21
XPT2046_Touchscreen ts(TS_CS);

// Touch calibration
#define TS_MINX 200
#define TS_MAXX 3800
#define TS_MINY 200
#define TS_MAXY 3800

// Timer
int totalSeconds = 30 * 60;
int remainingSeconds = 30 * 60;
bool isRunning = false;

// Sliders
int flow = 0;
int speed = 0;
int angle = 0;

// Buttons
struct Button { int x, y, w, h; const char* label; bool active; };
Button startBtn = {30, 230, 60, 40, "Start", false};
Button stopBtn  = {120, 230, 60, 40, "Stop", false};
Button flushBtn = {10, 30, 60, 30, "Flush", false};
Button oilBtn   = {80, 30, 60, 30, "Oil", false};
Button waterBtn = {150, 30, 60, 30, "Water", false};

// Sliders
struct Slider {
  int x, y, w, h;
  int maxVal;
  int* val;
  const char* label;
};
Slider sliders[3] = {
  {30, 80, 200, 30, 30, &flow, "Flow"},
  {30, 130, 200, 30, 30, &speed, "Speed"},
  {30, 180, 200, 30, 90, &angle, "Angle"}
};

// ---------------- Draw Slider ----------------
void drawSlider(Slider s) {
  tft.fillRect(s.x, s.y, s.w, s.h, TFT_DARKGREY);
  int fill = map(*s.val, 0, s.maxVal, 0, s.w);
  tft.fillRect(s.x, s.y, fill, s.h, TFT_BLUE);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setCursor(s.x, s.y - 16);
  tft.print(s.label);

  tft.setCursor(s.x + s.w + 5, s.y + s.h/2 - 8);
  tft.print(*s.val);
}

// ---------------- Draw Button ----------------
void drawButton(Button btn, uint16_t color) {
  tft.fillRect(btn.x, btn.y, btn.w, btn.h, color);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(btn.x + 5, btn.y + btn.h/2 - 8);
  tft.print(btn.label);
}

// ---------------- Draw Timer ----------------
void drawTimer() {
  int minutes = remainingSeconds / 60;
  int seconds = remainingSeconds % 60;
  char buf[6];
  sprintf(buf, "%02d:%02d", minutes, seconds);

  int timerX = 90;
  int timerY = 280;
  int timerW = 120;
  int timerH = 40;

  tft.fillRect(timerX, timerY, timerW, timerH, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(timerX + 15, timerY + 10);
  tft.print(buf);
}

// ---------------- Touch Handlers ----------------
void checkSliderTouch(int px, int py) {
  px = constrain(px, 0, tft.width() - 1);
  for (int i = 0; i < 3; i++) {
    Slider s = sliders[i];
    if (py >= s.y && py <= s.y + s.h) {
      int newVal = map(px, s.x, s.x + s.w, 0, s.maxVal);
      newVal = constrain(newVal, 0, s.maxVal);
      *s.val = newVal;
      drawSlider(s);

      pumpSpeed = map(flow, 0, 30, 0, 255);
      stepsPerSec = map(speed, 0, 30, 0, 7000);
      oscAngleDeg = constrain(angle, 0, 90);
      stepsToMove = (PULSES_PER_REV * oscAngleDeg) / 360;
    }
  }
}

void checkButtonTouch(int px, int py) {
  if (px >= startBtn.x && px <= startBtn.x + startBtn.w &&
      py >= startBtn.y && py <= startBtn.y + startBtn.h) {
    isRunning = true;
    stopFlag = false;
  }

  if (px >= stopBtn.x && px <= stopBtn.x + stopBtn.w &&
      py >= stopBtn.y && py <= stopBtn.y + stopBtn.h) {
    isRunning = false;
    stopFlag = true;
    analogWrite(PUMP_PIN, 0);
    drawTimer();
  }

  Button* btns[] = {&flushBtn, &oilBtn, &waterBtn};
  for (int i = 0; i < 3; i++) {
    Button* b = btns[i];
    if (px >= b->x && px <= b->x + b->w &&
        py >= b->y && py <= b->y + b->h) {
      flushBtn.active = (i == 0);
      oilBtn.active   = (i == 1);
      waterBtn.active = (i == 2);

      if (i == 0) { flow = 10; speed = 10; angle = 10; }
      if (i == 1) { flow = 20; speed = 15; angle = 20; }
      if (i == 2) { flow = 25; speed = 20; angle = 30; }

      pumpSpeed = map(flow, 0, 30, 0, 255);
      stepsPerSec = map(speed, 0, 30, 0, 7000);
      oscAngleDeg = angle;
      stepsToMove = (PULSES_PER_REV * oscAngleDeg) / 360;
    }
  }
}

// ==================== BACKEND FUNCTIONS ====================
void checkBTConnection() {
  static bool lastConnected = false;
  bool connected = SerialBT.hasClient();
  if (connected != lastConnected) { stopFlag = false; lastConnected = connected; }
}

// ------------------- Bluetooth Handling -------------------
void readBTBuffered() {
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();

    if (c=='B' || c == '\n' || c == '\r') {
      btBuffer.trim();
      if (btBuffer.length() > 0) {
        processBTCommand(btBuffer);
        btBuffer = "";
      }
    } else {
      btBuffer += c;
    }
  }
}

void processBTCommand(String input) {
  input.trim();

  if (input.equalsIgnoreCase("START")) {
    stopFlag = false; isRunning = true;
    Serial.println("System STARTED via Bluetooth");
  } 
  else if (input.equalsIgnoreCase("STOP")) {
    stopFlag = true; isRunning = false;
    analogWrite(PUMP_PIN, 0);
    Serial.println("System STOPPED via Bluetooth");
  } 
  else if (input.startsWith("SPEED")) {
    int val = input.substring(5).toInt();
    val = constrain(val, 0, 30);
    speed = val;
    stepsPerSec = map(speed, 0, 30, 0, 7000);
    drawSlider(sliders[1]);
    Serial.print("Stepper speed updated to: "); Serial.println(speed);
  }
  else if (input.equalsIgnoreCase("STATUS")) {
    Serial.print("Temp: "); Serial.print(currentTemp);
    Serial.print(" °C | Heater: "); Serial.print(heaterState ? "ON" : "OFF");
    Serial.print(" | Pump: "); Serial.println(pumpRunning ? "ON" : "OFF");
  }
}

// ------- Serial Monitor Input -------
void readSerialInput() {
  static String serialBuffer = "";

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      serialBuffer.trim();
      if (serialBuffer.length() > 0) {
        processBTCommand(serialBuffer); // reuse same parser
        serialBuffer = "";
      }
    } else serialBuffer += c;
  }
}

// ------------------- Heater Control -------------------
void controlHeater() {
  static unsigned long lastRead = 0;
  if (millis() - lastRead > 1000) {
    sensors.requestTemperatures();
    currentTemp = sensors.getTempCByIndex(0);

    if (currentTemp == DEVICE_DISCONNECTED_C) {
      digitalWrite(HEATER_PIN, LOW);
      heaterState = false;
    } 
    else if (flushBtn.active || oilBtn.active || waterBtn.active) {
      digitalWrite(HEATER_PIN, HIGH);
      heaterState = true;
    } 
    else {
      if (currentTemp < 40.0) {
        digitalWrite(HEATER_PIN, HIGH);
        heaterState = true;
      } else {
        digitalWrite(HEATER_PIN, LOW);
        heaterState = false;
      }
    }

    Serial.print("Temp: ");
    if (currentTemp == DEVICE_DISCONNECTED_C) Serial.println("Sensor Error");
    else {
      Serial.print(currentTemp);
      Serial.print(" °C, Heater: "); Serial.print(heaterState ? "ON" : "OFF");
      Serial.print(", Pump: "); Serial.println(pumpRunning ? "ON" : "OFF");
    }

    lastRead = millis();
  }
}

// ------------------- Pump Control -------------------
void controlPumpByTemp() {
  static unsigned long lastRead = 0;
  if (millis() - lastRead > 1000) {
    sensors.requestTemperatures();
    float temp = sensors.getTempCByIndex(0);

    if (flushBtn.active || oilBtn.active || waterBtn.active) {
      if (!pumpRunning && !stopFlag) { pumpRunning = true; analogWrite(PUMP_PIN, pumpSpeed);}
      if (pumpRunning) analogWrite(PUMP_PIN, pumpSpeed);
      return;
    }

    if (temp != DEVICE_DISCONNECTED_C) {
      if (temp >= 38.0 && !pumpRunning && !stopFlag) { pumpRunning = true; analogWrite(PUMP_PIN, pumpSpeed);}
      if (stopFlag && pumpRunning) { pumpRunning = false; analogWrite(PUMP_PIN, 0);}
      if (pumpRunning) analogWrite(PUMP_PIN, pumpSpeed);
    }

    lastRead = millis();
  }
}

// ------------------- Stepper Motor -------------------
void setDirection(bool dir) {
  if (dir != currentDir) {
    digitalWrite(DIR_PIN, dir ? HIGH : LOW);
    currentDir = dir;
    delayMicroseconds(10);
  }
}

void stepMotor() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(STEP_PIN, LOW);
}

// ------------------- Stepper Oscillation -------------------
void oscillateStepper() {
  if (stopFlag || stepsPerSec <= 0) return;
  unsigned long now = micros();
  unsigned long stepDelay = 1000000L / stepsPerSec;
  if ((long)(now - lastStepTime) < (long)stepDelay) return;

  stepMotor();
  lastStepTime = now;

  switch (oscState) {
    case TO_POS:
      currentStepPos++;
      if (currentStepPos >= stepsToMove) { oscState = TO_ZERO_FROM_POS; setDirection(false); }
      break;
    case TO_ZERO_FROM_POS:
      currentStepPos--;
      if (currentStepPos <= 0) { oscState = TO_NEG; setDirection(false); }
      break;
    case TO_NEG:
      currentStepPos--;
      if (currentStepPos <= -stepsToMove) { oscState = TO_ZERO_FROM_NEG; setDirection(true); }
      break;
    case TO_ZERO_FROM_NEG:
      currentStepPos++;
      if (currentStepPos >= 0) { oscState = TO_POS; setDirection(true); }
      break;
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT");

  pinMode(STEP_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, currentDir); digitalWrite(STEP_PIN, LOW);

  pinMode(HEATER_PIN, OUTPUT); digitalWrite(HEATER_PIN, LOW);
  pinMode(PUMP_PIN, OUTPUT); analogWrite(PUMP_PIN, 0);

  sensors.begin(); sensors.setWaitForConversion(false);

  tft.init(); tft.setRotation(0);
  ts.begin(); ts.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  for (int i = 0; i < 3; i++) drawSlider(sliders[i]);
  drawButton(flushBtn, flushBtn.active ? TFT_ORANGE : TFT_DARKGREY);
  drawButton(oilBtn, oilBtn.active ? TFT_ORANGE : TFT_DARKGREY);
  drawButton(waterBtn, waterBtn.active ? TFT_ORANGE : TFT_DARKGREY);
  drawButton(startBtn, TFT_GREEN);
  drawButton(stopBtn, TFT_RED);
  drawTimer();

  Serial.println("System Ready. Commands:");
  Serial.println("SPEED 20 | START | STOP | STATUS");
}

// ==================== LOOP ====================
void loop() {
  static unsigned long lastUpdate = 0;

  if (isRunning && millis() - lastUpdate > 1000) {
    lastUpdate = millis();
    remainingSeconds--;
    if (remainingSeconds <= 0) { remainingSeconds = totalSeconds; isRunning = false; }
    drawTimer();
  }

  if (ts.touched()) {
    int sumX = 0, sumY = 0;
    for (int i = 0; i < 10; i++) {
      TS_Point p = ts.getPoint();
      sumX += p.x; sumY += p.y;
      delay(1);
    }
    int px = map(sumX / 10, TS_MINX, TS_MAXX, 0, tft.width());
    int py = map(sumY / 10, TS_MINY, TS_MAXY, 0, tft.height());

    checkSliderTouch(px, py);
    checkButtonTouch(px, py);

    for (int i = 0; i < 3; i++) drawSlider(sliders[i]);
    drawButton(flushBtn, flushBtn.active ? TFT_ORANGE : TFT_DARKGREY);
    drawButton(oilBtn, oilBtn.active ? TFT_ORANGE : TFT_DARKGREY);
    drawButton(waterBtn, waterBtn.active ? TFT_ORANGE : TFT_DARKGREY);
    drawButton(startBtn, TFT_GREEN);
    drawButton(stopBtn, TFT_RED);
    drawTimer();
  }

  // Send temperature over Bluetooth every 2 seconds
  static unsigned long lastBTTempSend = 0;
  if (millis() - lastBTTempSend > 2000) {
    lastBTTempSend = millis();
    if (SerialBT.hasClient()) {
      SerialBT.print("Temp: "); SerialBT.print(currentTemp);
      SerialBT.print(" °C | Heater: "); SerialBT.println(heaterState ? "ON" : "OFF");
    }
  }

  checkBTConnection();
  readBTBuffered();
  readSerialInput();
  controlHeater();
  controlPumpByTemp();
  oscillateStepper();
}