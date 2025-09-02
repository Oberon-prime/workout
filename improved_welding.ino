#include <GyverTimers.h>
#include <LiquidCrystal_I2C.h>

// ==== Аппаратные пины ====
#define BUTTON_PIN A2
#define TRIAC_PIN 10
#define ZC_PIN 2
#define VOLTAGE_PIN A6
#define CURRENT_PIN A7
#define SVARKA_LED A0

// ==== Константы системы ====
#define MIN_DELAY_TICKS 10
#define MAX_DELAY_TICKS 900
#define WELD_TIME_MS 20000UL
#define TARGET_VOLTAGE 32.0f
#define CORRECTION_INTERVAL 100UL
#define DISPLAY_INTERVAL 1000UL
#define AVG_SAMPLES 8

// ==== Пороги ошибок для адаптивного ПИД ====
#define ERROR_FAST_THRESHOLD 3.0f    // >3В - быстрая фаза
#define ERROR_MED_THRESHOLD 0.5f     // 0.5-3В - средняя фаза
#define ERROR_FINE_THRESHOLD 0.1f    // <0.1В - точная фаза

// ==== Глобальные переменные ====
unsigned long startTime;
int16_t delayTicks = 800;
float lastVoltage = 0;
float lastCurrent = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);

// ==== ПИД переменные ====
float pidError = 0;
float pidLastError = 0;
float pidIntegral = 0;
float pidDerivative = 0;
uint8_t pidPhase = 0; // 0=быстрая, 1=средняя, 2=точная

// ==== Быстрый constrain ====
static inline int16_t fastConstrain(int16_t val, int16_t min_val, int16_t max_val) {
  return (val < min_val) ? min_val : (val > max_val) ? max_val : val;
}

// ==== Адаптивный ПИД регулятор ====
float adaptivePID(float error) {
  float absError = fabs(error);
  float Kp, Ki, Kd;
  
  // Определяем фазу работы и коэффициенты
  if (absError > ERROR_FAST_THRESHOLD) {
    // БЫСТРАЯ ФАЗА: агрессивный P + небольшой D
    pidPhase = 0;
    Kp = 4.0f;
    Ki = 0.0f;
    Kd = 0.8f;
    pidIntegral = 0; // Сброс интеграла при больших ошибках
  }
  else if (absError > ERROR_MED_THRESHOLD) {
    // СРЕДНЯЯ ФАЗА: классический PID
    pidPhase = 1;
    Kp = 2.5f;
    Ki = 0.3f;
    Kd = 0.5f;
  }
  else {
    // ТОЧНАЯ ФАЗА: малый P + большой I для устранения статической ошибки
    pidPhase = 2;
    Kp = 1.2f;
    Ki = 0.8f;
    Kd = 0.3f;
  }
  
  // Вычисляем составляющие ПИД
  pidDerivative = error - pidLastError;
  pidIntegral += error;
  
  // Ограничиваем интеграл от накопления
  pidIntegral = constrain(pidIntegral, -50.0f, 50.0f);
  
  // Итоговая коррекция
  float correction = -(Kp * error + Ki * pidIntegral + Kd * pidDerivative);
  
  pidLastError = error;
  return correction;
}

// ==== Обработчик перехода через ноль ====
void onZC() {
  Timer1.setPeriod(delayTicks * 10);
  Timer1.enableISR(CHANNEL_A);
}

// ==== Обработчик таймера — импульс на симистор ====
ISR(TIMER1_A) {
  digitalWrite(TRIAC_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(TRIAC_PIN, LOW);
  Timer1.stop();
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(TRIAC_PIN, OUTPUT);
  digitalWrite(TRIAC_PIN, LOW);
  pinMode(ZC_PIN, INPUT_PULLUP);
  pinMode(SVARKA_LED, OUTPUT);

  Serial.begin(9600);

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

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BUTTON_PIN) == LOW) {
      startWelding();
    }
  }
}

void startWelding() {
  startTime = millis();
  delayTicks = 800;
  
  // Сброс ПИД переменных
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

  // Буфер для усреднения (уменьшен с 10 до 8)
  float voltageBuffer[AVG_SAMPLES] = {0};
  uint8_t bufIndex = 0;
  bool bufferFilled = false;

  while (millis() - startTime < WELD_TIME_MS) {
    unsigned long now = millis();

    // === Коррекция каждые 100мс ===
    if (now - lastCorrection >= CORRECTION_INTERVAL) {
      lastCorrection = now;

      // Читаем напряжение (пока простое измерение, потом заменим на RMS)
      float voltage = analogRead(VOLTAGE_PIN) * (5.0f / 1023.0f) * 10.1f;
      lastVoltage = voltage;

      // Сохраняем в буфер для дисплея
      voltageBuffer[bufIndex] = voltage;
      bufIndex = (bufIndex + 1) % AVG_SAMPLES;
      if (bufIndex == 0) bufferFilled = true;

      // === АДАПТИВНЫЙ ПИД РЕГУЛЯТОР ===
      pidError = voltage - TARGET_VOLTAGE;
      float correction = adaptivePID(pidError);
      
      // Применяем коррекцию
      int16_t corrInt = (int16_t)round(correction);
      
      // Минимальный шаг при малых ошибках
      if (corrInt == 0 && fabs(pidError) > 0.05f) {
        corrInt = (correction > 0) ? 1 : -1;
      }

      delayTicks = fastConstrain(delayTicks + corrInt, MIN_DELAY_TICKS, MAX_DELAY_TICKS);

      // Измеряем ток
      lastCurrent = analogRead(CURRENT_PIN) * (5.0f / 1023.0f) * 10.1f;
    }

    // === Обновление дисплея каждую секунду ===
    if (now - lastDisplay >= DISPLAY_INTERVAL) {
      lastDisplay = now;

      // Усредняем напряжение для показа
      uint8_t count = bufferFilled ? AVG_SAMPLES : bufIndex;
      float sum = 0;
      for (uint8_t i = 0; i < count; i++) sum += voltageBuffer[i];
      float avgVoltage = sum / (float)count;

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("U:");
      lcd.print(avgVoltage, 1);
      lcd.print("V Zad:");
      lcd.print(TARGET_VOLTAGE, 0);
      lcd.print("V");
      
      lcd.setCursor(0, 1);
      lcd.print("Err:");
      lcd.print(pidError, 2);
      lcd.print("V Phase:");
      lcd.print(pidPhase);
      
      lcd.setCursor(0, 2);
      lcd.print("Delay:");
      lcd.print(delayTicks * 10);
      lcd.print("us I:");
      lcd.print(lastCurrent, 1);
      lcd.print("A");
      
      lcd.setCursor(0, 3);
      lcd.print("Time:");
      lcd.print((WELD_TIME_MS - (now - startTime)) / 1000);
      lcd.print("s");
    }
  }

  // Завершение сварки
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