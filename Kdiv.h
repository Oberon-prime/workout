#pragma once
#include <stdint.h>
#include <avr/pgmspace.h>

// === Параметры сети ===
#ifndef MAINS_FREQ_HZ
#define MAINS_FREQ_HZ 50.0f
#endif
#ifndef DELAY_TICK_US
#define DELAY_TICK_US 10.0f
#endif

// Методы коррекции
#define METHOD_TABLE 0
#define METHOD_POLY 1

// === Оптимизированные константы ===
static const float INV_PERIOD = 2.0f * MAINS_FREQ_HZ * (DELAY_TICK_US * 1e-6f); // Предвычисленная константа

// Быстрое преобразование delayTicks -> x
static inline float delayTicks_to_x(uint16_t delayTicks) {
  return INV_PERIOD * (float)delayTicks;
}

// === МЕТОД 1: Таблица интерполяции ===
static const uint8_t NX = 15;

// Сетка X (в PROGMEM для экономии RAM)
static const float PROGMEM X_GRID[NX] = {
  0.010f, 0.086f, 0.110f, 0.171f, 0.240f,
  0.260f, 0.327f, 0.354f, 0.376f, 0.460f,
  0.482f, 0.493f, 0.595f, 0.611f, 0.615f
};

// Коэффициенты Kdiv для разных токов
static const float PROGMEM KDIV_I0[NX] = {
  0.877193f, 0.837757f, 0.825303f, 0.793651f, 0.772693f,
  0.766619f, 0.746269f, 0.724075f, 0.705991f, 0.636943f,
  0.614626f, 0.603468f, 0.500000f, 0.500000f, 0.500000f
};

static const float PROGMEM KDIV_I1[NX] = {
  0.858369f, 0.858369f, 0.841773f, 0.799593f, 0.751880f,
  0.738511f, 0.693724f, 0.675676f, 0.658894f, 0.594817f,
  0.578035f, 0.565342f, 0.447647f, 0.429185f, 0.429185f
};

static const float PROGMEM KDIV_I2[NX] = {
  0.836820f, 0.836820f, 0.836820f, 0.805531f, 0.770137f,
  0.759878f, 0.703491f, 0.680767f, 0.662252f, 0.599403f,
  0.582943f, 0.574713f, 0.488586f, 0.475076f, 0.471698f
};

// Быстрое чтение из PROGMEM
static inline float pgm_read_f(const float* p) { 
  return pgm_read_float_near(p); 
}

// Оптимизированная линейная интерполяция
static inline float interp1_x(const float* X, const float* Y, float x) {
  // Граничные случаи
  float x0 = pgm_read_f(&X[0]);
  if (x <= x0) return pgm_read_f(&Y[0]);
  
  float xN = pgm_read_f(&X[NX-1]);
  if (x >= xN) return pgm_read_f(&Y[NX-1]);

  // Поиск интервала (можно заменить на бинарный поиск для больших таблиц)
  uint8_t i;
  float xi, xi1;
  for (i = 0; i < NX-1; i++) {
    xi = pgm_read_f(&X[i]);
    xi1 = pgm_read_f(&X[i+1]);
    if (x <= xi1) break;
  }
  
  // Линейная интерполяция
  float yi = pgm_read_f(&Y[i]);
  float yi1 = pgm_read_f(&Y[i+1]);
  float t = (x - xi) / (xi1 - xi);
  return yi + (yi1 - yi) * t;
}

// Получение Kdiv по таблице
static inline float Kdiv_table(float x, uint8_t I_idx) {
  switch(I_idx) {
    case 0:  return interp1_x(X_GRID, KDIV_I0, x);
    case 1:  return interp1_x(X_GRID, KDIV_I1, x);
    default: return interp1_x(X_GRID, KDIV_I2, x);
  }
}

// Основная функция коррекции через таблицу
static inline float Urms_from_Uavg_table(float Uavg, uint16_t delayTicks, uint8_t I_idx) {
  float x = delayTicks_to_x(delayTicks);
  float Kdiv = Kdiv_table(x, I_idx);
  return Uavg / Kdiv;
}

// === МЕТОД 2: Полиномиальная аппроксимация ===
// Коэффициенты квадратичной поверхности K_div(x, I_idx)
static const float c0 =  0.88414327f;
static const float c1 = -0.28701208f;
static const float c2 = -0.04083378f;
static const float c3 = -0.59561809f;
static const float c4 =  0.02119531f;
static const float c5 = -0.02618245f;

// Вычисление Kdiv полиномом
static inline float Kdiv_poly(float x, float I_idx) {
  // Ограничиваем x допустимым диапазоном
  if (x < 0.01f) x = 0.01f;
  if (x > 0.62f) x = 0.62f;
  
  // Вычисляем полином: c0 + c1*x + c2*I + c3*x² + c4*I² + c5*x*I
  float x2 = x * x;
  float I2 = I_idx * I_idx;
  
  return c0 + c1*x + c2*I_idx + c3*x2 + c4*I2 + c5*x*I_idx;
}

// Основная функция коррекции через полином
static inline float Urms_from_Uavg_poly(float Uavg, uint16_t delayTicks, float I_idx) {
  float x = delayTicks_to_x(delayTicks);
  float Kdiv = Kdiv_poly(x, I_idx);
  
  // Защита от деления на ноль/малые значения
  if (Kdiv < 0.1f) Kdiv = 0.1f;
  
  return Uavg / Kdiv;
}