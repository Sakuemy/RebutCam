/*
  Поминутное усреднение A0 и A2
  Выход D12 — если хотя бы один канал ниже порога в конце минуты
  В начале новой минуты EMA сбрасывается (холодный старт)

  + Раз в сутки выход D12 принудительно включается на outputTime (5 с) —
    плановая перезагрузка камер. После неё мониторинг приостанавливается
    на bootGraceTime, чтобы камеры успели загрузиться и не дали ложного
    срабатывания.
*/

#include <avr/wdt.h>

const byte analogPins[2] = {A0, A2};
const byte outputPin = 12;

// ===== НАСТРОЙКИ =====
const float referenceVoltage = 5.0;
const float thresholdVoltage = 0.15;
const float alpha = 0.002;
const uint32_t MINUTE_MS = 60000UL;
const unsigned long outputTime = 5000;              // длительность отключения, мс

const unsigned long dailyInterval = 86400000UL;     // 24 ч (для теста можно 120000UL)
const unsigned long bootGraceTime = 60000UL;        // пауза после суточного отключения, мс

#define USE_WATCHDOG 1   // 0 — выключить (см. примечание про старый загрузчик Nano)
// ======================

// ===== СОСТОЯНИЕ EMA =====
float emaValue[2] = {512.0, 512.0};
bool emaReady[2] = {false, false};
uint16_t samplesDone[2] = {0, 0};

// ===== ТАЙМЕРЫ =====
unsigned long lastSampleTime = 0;
unsigned long minuteStart = 0;
unsigned long outputStartTime = 0;
unsigned long lastDailyCut = 0;
unsigned long graceStart = 0;
bool outputActive = false;
bool dailyCycle = false;     // текущий импульс выхода — суточный
bool graceActive = false;    // ждём загрузки камер после суточного отключения

const unsigned long sampleInterval = 100; // мс между семплами

// Порог в единицах АЦП
uint16_t thresholdADC;

// Чтение с холостым измерением: после смены канала мультиплексору нужно устояться
uint16_t readADC(byte pin) {
  analogRead(pin);
  return analogRead(pin);
}

// ===== СБРОС EMA (начало новой минуты) =====
void resetEMA() {
  for (uint8_t ch = 0; ch < 2; ch++) {
    emaValue[ch] = (float)readADC(analogPins[ch]);
    emaReady[ch] = false;
    samplesDone[ch] = 0;
  }
}

// Полный перезапуск минутного окна (после паузы)
void startMinute(unsigned long now) {
  resetEMA();
  minuteStart = now;
  lastSampleTime = now;
}

void setup() {
  MCUSR = 0;
  wdt_disable();

  Serial.begin(9600);
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, LOW);

  thresholdADC = (uint16_t)(thresholdVoltage * 1023.0 / referenceVoltage);

  unsigned long now = millis();
  startMinute(now);
  lastDailyCut = now;

  Serial.println(F("Ready — minute averaging, 2 channels, daily cut"));

#if USE_WATCHDOG
  wdt_enable(WDTO_8S);
#endif
}

void loop() {
#if USE_WATCHDOG
  wdt_reset();
#endif
  unsigned long now = millis();

  // ----- Выключение выхода по таймеру -----
  if (outputActive && (now - outputStartTime >= outputTime)) {
    digitalWrite(outputPin, LOW);
    outputActive = false;
    Serial.println(F("OUTPUT OFF"));

    if (dailyCycle) {
      dailyCycle = false;
      graceActive = true;
      graceStart = now;
      Serial.println(F("Daily cut done — waiting for cameras to boot"));
    }
  }

  // ----- Во время суточного отключения мониторинг не ведём -----
  if (dailyCycle) return;

  // ----- Пауза после суточного отключения -----
  if (graceActive) {
    if (now - graceStart >= bootGraceTime) {
      graceActive = false;
      startMinute(now);
      Serial.println(F("Grace over — monitoring resumed"));
    }
    return;
  }

  // ----- Суточное отключение (не накладываем на аварийный импульс) -----
  if (!outputActive && (now - lastDailyCut >= dailyInterval)) {
    lastDailyCut += dailyInterval;
    digitalWrite(outputPin, HIGH);
    outputStartTime = now;
    outputActive = true;
    dailyCycle = true;
    Serial.println(F("DAILY CUT — cameras off"));
    return;
  }

  // ----- Семплирование обоих каналов -----
  if (now - lastSampleTime >= sampleInterval) {
    lastSampleTime = now;   // без накопления: после паузы не будет «пачки» семплов

    for (uint8_t ch = 0; ch < 2; ch++) {
      uint16_t raw = readADC(analogPins[ch]);
      emaValue[ch] += alpha * ((float)raw - emaValue[ch]);
      samplesDone[ch]++;

      // Считаем EMA «прогретым» после 500 семплов (~50 с при 100 мс)
      if (!emaReady[ch] && samplesDone[ch] >= 500) {
        emaReady[ch] = true;
      }
    }
  }

  // ----- Конец минуты — анализ и сброс -----
  if (now - minuteStart >= MINUTE_MS) {
    minuteStart += MINUTE_MS;

    // Печатаем итог минуты
    for (uint8_t ch = 0; ch < 2; ch++) {
      float v = emaValue[ch] * referenceVoltage / 1023.0;
      Serial.print(F("CH")); Serial.print(ch);
      Serial.print(F(" end-of-minute avg: ")); Serial.print(v, 3);
      Serial.println(F(" V"));
    }

    // Срабатываем, если оба EMA прогреты и хотя бы один ниже порога
    bool ch0Low = emaReady[0] && ((uint16_t)emaValue[0] < thresholdADC);
    bool ch1Low = emaReady[1] && ((uint16_t)emaValue[1] < thresholdADC);

    if (ch0Low || ch1Low) {
      if (!outputActive) {
        digitalWrite(outputPin, HIGH);
        outputStartTime = now;
        outputActive = true;
        Serial.print(F("TRIGGER — low on:"));
        if (ch0Low) Serial.print(F(" CH0"));
        if (ch1Low) Serial.print(F(" CH1"));
        Serial.println();
      }
    }

    // Сброс EMA в начале новой минуты
    resetEMA();
    Serial.println(F("--- new minute, EMA reset ---"));
  }
}
