#include <GyverTimers.h>
#include <LiquidCrystal_I2C.h>
#include "Kdiv.h"

// === Настройки метода коррекции ===
#define USE_METHOD METHOD_TABLE
//#define USE_METHOD METHOD_POLY

// === Аппаратные пины ===
#define BUTTON_PIN A2
#define TRIAC_PIN 10
#define ZC_PIN 2
#define VOLTAGE_PIN A6
#define CURRENT_PIN A7
#define SVARKA_LED A0

// === Константы системы ===
#define MIN_DELAY_TICKS 10
#define MAX_DELAY_TICKS 900
#define WELD_TIME_MS 20000UL
#define TARGET_VOLTAGE 32.0f
#define CORRECTION_INTERVAL 100UL
#define DISPLAY_INTERVAL 1000UL
#define AVG_SAMPLES 8

// === Константы АЦП ===
const float ADC_SCALE = (5.0f / 1023.0f) * 10.1f;

// === Пороги ошибок для адаптивного ПИД ===
#define ERROR_FAST_THRESHOLD 3.0f
#define ERROR_MED_THRESHOLD 0.5f
#define ERROR_FINE_THRESHOLD 0.1f

// === Типы тока ===
enum CurrentIndex {
  LOW_CURRENT = 0,
  MED_CURRENT = 1,
  HIGH_CURRENT = 2
};

// === Глобальные переменные ===
unsigned long startTime;
int16_t delayTicks = 800;
float lastVoltage = 0;
float lastCurrent = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);

// === ПИД переменные ===
float pidError = 0;
float pidLastError = 0;
float pidIntegral = 0;
float pidDerivative = 0;
uint8_t pidPhase = 0;

// === Выбор индекса тока по сопротивлению ===
static inline uint8_t selectCurrentIndex(float R_nom) {
  if (R_nom > 6.0f) return LOW_CURRENT;
  if (R_nom > 1.8f) return MED_CURRENT;
  return HIGH_CURRENT;
}

// === Обёртка для коррекции Uavg -> Urms ===
static inline float calcUrmsFromUavg(float Uavg, uint16_t delayTicks_local, uint8_t I_idx) {
#if (USE_METHOD == METHOD_TABLE)
  return Urms_from_Uavg_table(Uavg, delayTicks_local, I_idx);
#else
  return Urms_from_Uavg_poly(Uavg, delayTicks_local, (float)I_idx);
#endif
}

// === Быстрый constrain ===
static inline int16_t fastConstrain(int16_t val, int16_t min_val, int16_t max_val) {
  return (val < min_val) ? min_val : (val > max_val) ? max_val : val;
}

// === Адаптивный ПИД регулятор ===
float adaptivePID(float error) {
  float absError = fabs(error);
  float Kp, Ki, Kd;
  
  if (absError > ERROR_FAST_THRESHOLD) {
    pidPhase = 0;
    Kp = 4.0f;
    Ki = 0.0f;
    Kd = 0.8f;
    pidIntegral = 0;
  }
  else if (absError > ERROR_MED_THRESHOLD) {
    pidPhase = 1;
    Kp = 2.5f;
    Ki = 0.3f;
    Kd = 0.5f;
  }
  else {
    pidPhase = 2;
    Kp = 1.2f;
    Ki = 0.8f;
    Kd = 0.3f;
  }
  
  pidDerivative = error - pidLastError;
  pidIntegral += error;
  pidIntegral = constrain(pidIntegral, -50.0f, 50.0f);
  
  float correction = -(Kp * error + Ki * pidIntegral + Kd * pidDerivative);
  pidLastError = error;
  return correction;
}

// === Обработчики прерываний ===
void onZC() {
  Timer1.setPeriod(delayTicks * 10);
  Timer1.enableISR(CHANNEL_A);
}

ISR(TIMER1_A) {
  digitalWrite(TRIAC_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(TRIAC_PIN, LOW);
  Timer1.stop();
}

// === Инициализация ===
void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(TRIAC_PIN, OUTPUT);
  digitalWrite(TRIAC_PIN, LOW);
  pinMode(ZC_PIN, INPUT_PULLUP);
  pinMode(SVARKA_LED, OUTPUT);

  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SVARKA v2.0"));
  lcd.setCursor(0, 1);
  lcd.print(F("Adaptive PID"));
  lcd.setCursor(0, 2);
  lcd.print(F("Press PUSK..."));
}

// === Главный цикл ===
void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BUTTON_PIN) == LOW) {
      startWelding();
    }
  }
}

// === Основная функция сварки ===
void startWelding() {
  startTime = millis();
  delayTicks = 800;
  
  pidError = 0;
  pidLastError = 0;
  pidIntegral = 0;
  pidPhase = 0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("WELDING ACTIVE"));
  digitalWrite(SVARKA_LED, HIGH);

  attachInterrupt(digitalPinToInterrupt(ZC_PIN), onZC, FALLING);

  unsigned long lastCorrection = millis();
  unsigned long lastDisplay = millis();

  float voltageBuffer[AVG_SAMPLES] = {0};
  uint8_t bufIndex = 0;
  bool bufferFilled = false;

  float R_nom = 8.2f;
  uint8_t currentIdx = selectCurrentIndex(R_nom);

  while (millis() - startTime < WELD_TIME_MS) {
    unsigned long now = millis();

    if (now - lastCorrection >= CORRECTION_INTERVAL) {
      lastCorrection = now;

      float Uavg = analogRead(VOLTAGE_PIN) * ADC_SCALE;
      float Urms_est = calcUrmsFromUavg(Uavg, (uint16_t)delayTicks, currentIdx);
      lastVoltage = Uavg;

      voltageBuffer[bufIndex] = Uavg;
      bufIndex = (bufIndex + 1) % AVG_SAMPLES;
      if (bufIndex == 0) bufferFilled = true;

      pidError = Urms_est - TARGET_VOLTAGE;
      float correction = adaptivePID(pidError);
      
      int16_t corrInt = (int16_t)round(correction);
      
      if (corrInt == 0 && fabs(pidError) > 0.05f) {
        corrInt = (correction > 0) ? 1 : -1;
      }

      delayTicks = fastConstrain(delayTicks + corrInt, MIN_DELAY_TICKS, MAX_DELAY_TICKS);
      lastCurrent = analogRead(CURRENT_PIN) * ADC_SCALE;
    }

    if (now - lastDisplay >= DISPLAY_INTERVAL) {
      lastDisplay = now;

      uint8_t count = bufferFilled ? AVG_SAMPLES : bufIndex;
      float sum = 0;
      for (uint8_t i = 0; i < count; i++) sum += voltageBuffer[i];
      float avgUavg = sum / (float)count;

      float Urms_show = calcUrmsFromUavg(avgUavg, (uint16_t)delayTicks, currentIdx);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("U:");
      lcd.print(avgUavg, 1);
      lcd.print("V Urms:");
      lcd.print(Urms_show, 1);
      lcd.print("V");
      
      lcd.setCursor(0, 1);
      lcd.print("Zad:");
      lcd.print(TARGET_VOLTAGE, 0);
      lcd.print("V Err:");
      lcd.print(pidError, 2);
      lcd.print("V");
      
      lcd.setCursor(0, 2);
      lcd.print("Delay:");
      lcd.print(delayTicks * 10);
      lcd.print("us P");
      lcd.print(pidPhase);
      
      lcd.setCursor(0, 3);
      lcd.print("I:");
      lcd.print(lastCurrent, 1);
      lcd.print("A Time:");
      lcd.print((WELD_TIME_MS - (millis() - startTime)) / 1000);
      lcd.print("s");
    }
  }

  digitalWrite(TRIAC_PIN, LOW);
  Timer1.stop();
  detachInterrupt(digitalPinToInterrupt(ZC_PIN));
  digitalWrite(SVARKA_LED, LOW);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("WELDING COMPLETE"));
  lcd.setCursor(0, 1);
  lcd.print(F("Time: 20 seconds"));
}