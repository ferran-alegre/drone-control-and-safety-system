#include <Arduino.h>
#include <LiquidCrystal.h>
#include <dht_nonblocking.h>
#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>  // Jarzebski: https://github.com/jarzebski/Arduino-MPU6050
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Target: Arduino Mega 2560, 16 x 2 LCD.
// Connect each button between its input pin and GND (INPUT_PULLUP).
// Pages: 0 = environment, 1 = distance, 2 = electrical telemetry, 3 = pitch & roll.

// =========================
// Pins and hardware
// =========================

const uint8_t SW_pin = 2;
const uint8_t X_pin = A0;
const uint8_t Y_pin = A1;
const uint8_t battery_pin = A2;
const uint8_t R_battery_pin = A3;
const uint8_t lcd_button = 53;
const uint8_t motorPwmPin = 5;
const uint8_t buzzerPin = 6;
const uint8_t TRIG_PIN = 4;
const uint8_t ECHO_PIN = 3;  // External interrupt input on the Mega.
const uint8_t DHT_SENSOR_PIN = 13;
const uint8_t RED_LED_PIN = 22;
const uint8_t GREEN_LED_PIN = 24;
const uint8_t BLUE_LED_PIN = 26;
const uint8_t SCL_PIN = 21;
const uint8_t SDA_PIN = 20;
const uint8_t CalibrationButton_PIN = 31;
const uint8_t SERVO_PIN = 28;  // Libre (el 11 del ejemplo lo usa el LCD como D6)

const uint8_t MPU6050_ADDR = 0x68;
MPU6050 mpu;
bool imuOk = false;
uint8_t imuFailCount = 0;
uint16_t imuErrors = 0;  // Lecturas fallidas acumuladas (diagnóstico de cableado)

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_TYPE_11);

const float ADC_REF = 5.0f;
const float ADC_MAX = 1023.0f;
const float R_SHUNT = 1.0f;  // Ohms
const float BATTERY_VOLTAGE_SCALE = 1.0f;
const float SHUNT_LOW_VOLTAGE_SCALE = 1.0f;
const float CURRENT_OFFSET_A = 0.0f;

// =========================
// Timing and state
// =========================

const uint32_t DEBOUNCE_MS = 50;
const uint32_t CURRENT_SAMPLE_MS = 10;
const uint8_t CURRENT_SAMPLE_COUNT = 20;
const uint8_t VOLTAGE_SAMPLE_COUNT = 20; // Promedio de muestras de voltaje
const uint32_t LCD_REFRESH_MS = 250;
const uint32_t DISTANCE_INTERVAL_MS = 500;
const uint32_t ECHO_TIMEOUT_US = 30000;
const uint8_t DISPLAY_COUNT = 4;

// Umbrales de batería (con histéresis para evitar oscilaciones por caída de tensión)
const float LOW_BATT_ENTER_V = 3.5f;  // Entra en modo aterrizaje por debajo de este valor
const float LOW_BATT_EXIT_V  = 3.7f;  // Sale del modo solo por encima de este valor

int safetyRawState = HIGH, safetyStableState = HIGH;
int lcdRawState = HIGH, lcdStableState = HIGH;
int calRawState = HIGH, calStableState = HIGH;
int calIdleState = HIGH;  // Nivel del pin 31 con el botón SIN pulsar (se detecta al arrancar)
uint32_t calLastChangeMs = 0;
bool calibrateRequested = false;
uint32_t safetyLastChangeMs = 0, lcdLastChangeMs = 0;
bool safety_switch = false;
bool motor_state = false;
int x_value = 0, y_value = 0, pwmValue = 0;

uint8_t displayMode = 0;
bool displayChanged = true;
uint32_t lastLcdUpdateMs = 0;
char lcdTarget[2][17];
char lcdWritten[2][17] = {};

// Promedio de voltaje y corriente
float battery_voltage = 0.0f;
float voltageSamples[VOLTAGE_SAMPLE_COUNT] = {};
float voltageSum = 0.0f;
float lowVoltageSamples[VOLTAGE_SAMPLE_COUNT] = {};  // Tensión del lado bajo del shunt
float lowVoltageSum = 0.0f;
uint8_t voltageSampleIndex = 0, voltageSampleCount = 0;
bool lowBatteryMode = false;

float battery_current = 0.0f;
float average_current = 0.0f;
float consumed_mAh = 0.0f;
float currentSamples[CURRENT_SAMPLE_COUNT] = {};
float currentSum = 0.0f;
float previousCurrent = 0.0f;
uint8_t sampleIndex = 0, sampleCount = 0;
uint32_t lastCurrentSampleMs = 0;

float temperature = 20.0f, humidity = 0.0f;
bool environmentValid = false;
float calibrated_distance = 0.0f;
bool distanceValid = false;
const float security_distance = 20.0f;

// GY-521 (MPU-6050) State
float pitch = 0.0f, roll = 0.0f;
float pitchOffset = 0.0f, rollOffset = 0.0f;
float maxInclinationValue = 0.0f;
const float MAX_TILT_DEG = 45.0f;  // Inclinación a la que el acelerador llega al máximo
float pitchSigned = 0.0f;  // Pitch con signo (respecto a la calibración), para el servo y el LCD
float rollSigned = 0.0f;   // Roll con signo (respecto a la calibración), para el LCD

// Servo que sigue el pitch: -45° -> 30°, 0° -> 90°, +45° -> 150°
Servo pitchServo;
const int SERVO_CENTER_DEG = 90;
const int SERVO_MIN_DEG = 30;
const int SERVO_MAX_DEG = 150;
const float SERVO_GAIN = 2.0f;            // Multiplicador del movimiento (1 = -45..45° -> 30..150°)
const bool SERVO_INVERT = false;           // true si el servo gira en sentido contrario al pitch
const uint32_t SERVO_UPDATE_MS = 20;       // Periodo de actualización del servo
const float SERVO_SMOOTHING = 0.25f;       // 0..1, menor = movimiento más suave
float servoAngle = SERVO_CENTER_DEG;
int servoCommand = SERVO_CENTER_DEG;
float incrementPWM = 0.0f;
float incrementPWM2 = 0.0f;

// Duración del ciclo completo del bucle (µs)
uint32_t loopTimeUs = 0, loopTimeMaxUs = 0;

enum SonarState { SONAR_IDLE, TRIGGER_LOW, TRIGGER_HIGH, WAITING_FOR_ECHO };
SonarState sonarState = SONAR_IDLE;
uint32_t lastDistanceUpdateMs = 0, sonarTimestampUs = 0;
volatile bool captureEcho = false, echoStarted = false, echoReady = false;
volatile uint32_t echoStartUs = 0, echoWidthUs = 0;

// =========================
// Debouncing
// =========================

bool updateButton(uint8_t pin, int &rawState, int &stableState,
                  uint32_t &lastChangeMs, uint32_t nowMs) {
  int reading = digitalRead(pin);
  if (reading != rawState) {
    rawState = reading;
    lastChangeMs = nowMs;
  }
  if ((uint32_t)(nowMs - lastChangeMs) >= DEBOUNCE_MS &&
      stableState != rawState) {
    stableState = rawState;
    return stableState == LOW;
  }
  return false;
}

// Botón de calibración que funciona con cualquier cableado:
// el nivel que lee el pin al arrancar se toma como "sin pulsar", y pulsar es cambiar a
// ese nivel contrario (botón a GND: pulsar = LOW; botón a 5V con pull-down: pulsar = HIGH).
// Devuelve true UNA vez por pulsación (con antirrebote).
bool calibrationPressed(uint32_t nowMs) {
  int reading = digitalRead(CalibrationButton_PIN);
  if (reading != calRawState) {
    calRawState = reading;
    calLastChangeMs = nowMs;
  }
  if ((uint32_t)(nowMs - calLastChangeMs) >= DEBOUNCE_MS && calStableState != calRawState) {
    calStableState = calRawState;
    return calStableState != calIdleState;
  }
  return false;
}

// =========================
// Measurement & Averaging
// =========================

void updateBattery(uint32_t nowMs) {
  uint32_t elapsedMs = nowMs - lastCurrentSampleMs;
  if (sampleCount > 0 && elapsedMs < CURRENT_SAMPLE_MS) return;
  // En la primera muestra no hay intervalo previo: evita integrar el tiempo desde el arranque.
  if (sampleCount == 0) elapsedMs = 0;

  float instVoltage = analogRead(battery_pin) * ADC_REF / ADC_MAX * BATTERY_VOLTAGE_SCALE;
  float instLowVoltage = analogRead(R_battery_pin) * ADC_REF / ADC_MAX * SHUNT_LOW_VOLTAGE_SCALE;

  // Promedio móvil de AMBAS tensiones (misma ventana) para estabilizar la lectura,
  // ya que con el motor en marcha el valor instantáneo oscila mucho (PWM).
  voltageSum -= voltageSamples[voltageSampleIndex];
  voltageSamples[voltageSampleIndex] = instVoltage;
  voltageSum += instVoltage;

  lowVoltageSum -= lowVoltageSamples[voltageSampleIndex];
  lowVoltageSamples[voltageSampleIndex] = instLowVoltage;
  lowVoltageSum += instLowVoltage;

  voltageSampleIndex = (voltageSampleIndex + 1) % VOLTAGE_SAMPLE_COUNT;
  if (voltageSampleCount < VOLTAGE_SAMPLE_COUNT) ++voltageSampleCount;
  battery_voltage = voltageSum / voltageSampleCount;
  float averageLowVoltage = lowVoltageSum / voltageSampleCount;

  // Corriente a partir de las dos tensiones promediadas (coherentes entre sí)
  battery_current = (battery_voltage - averageLowVoltage) / R_SHUNT - CURRENT_OFFSET_A;
  if (battery_current < 0.0f) battery_current = 0.0f;

  // Consumo acumulado (regla del trapecio): A * ms / 3600 = mAh
  if (sampleCount == 0) previousCurrent = battery_current;
  consumed_mAh += 0.5f * (previousCurrent + battery_current) * elapsedMs / 3600.0f;
  previousCurrent = battery_current;
  lastCurrentSampleMs = nowMs;

  currentSum -= currentSamples[sampleIndex];
  currentSamples[sampleIndex] = battery_current;
  currentSum += battery_current;
  sampleIndex = (sampleIndex + 1) % CURRENT_SAMPLE_COUNT;
  if (sampleCount < CURRENT_SAMPLE_COUNT) ++sampleCount;
  average_current = currentSum / sampleCount;

  // Modo batería baja con histéresis, solo cuando el promedio de voltaje ya es fiable.
  // Sin histéresis, al recortar el acelerador la tensión se recupera, se vuelve a
  // dar potencia, la tensión cae otra vez... y el motor oscila.
  if (voltageSampleCount >= VOLTAGE_SAMPLE_COUNT) {
    if (!lowBatteryMode && battery_voltage < LOW_BATT_ENTER_V) lowBatteryMode = true;
    else if (lowBatteryMode && battery_voltage > LOW_BATT_EXIT_V) lowBatteryMode = false;
  }
}

// =========================
// Ultrasonic Measurement
// =========================

void echoInterrupt() {
  if (!captureEcho) return;
  uint32_t nowUs = micros();
  if (digitalRead(ECHO_PIN) == HIGH) {
    if (!echoStarted) {
      echoStartUs = nowUs;
      echoStarted = true;
    }
  } else if (echoStarted) {
    echoWidthUs = nowUs - echoStartUs;
    echoReady = true;
    captureEcho = false;
  }
}

void updateDistance() {
  uint32_t nowUs = micros();
  switch (sonarState) {
    case SONAR_IDLE:
      if ((uint32_t)(millis() - lastDistanceUpdateMs) >= DISTANCE_INTERVAL_MS) {
        lastDistanceUpdateMs = millis();
        digitalWrite(TRIG_PIN, LOW);
        sonarTimestampUs = micros();
        sonarState = TRIGGER_LOW;
      }
      break;

    case TRIGGER_LOW:
      if ((uint32_t)(nowUs - sonarTimestampUs) >= 2) {
        digitalWrite(TRIG_PIN, HIGH);
        sonarTimestampUs = micros();
        sonarState = TRIGGER_HIGH;
      }
      break;

    case TRIGGER_HIGH:
      if ((uint32_t)(nowUs - sonarTimestampUs) >= 10) {
        noInterrupts();
        echoStarted = false;
        echoReady = false;
        captureEcho = true;
        interrupts();
        digitalWrite(TRIG_PIN, LOW);
        sonarTimestampUs = micros();
        sonarState = WAITING_FOR_ECHO;
      }
      break;

    case WAITING_FOR_ECHO: {
      noInterrupts();
      bool ready = echoReady;
      uint32_t widthUs = echoWidthUs;
      if (ready) echoReady = false;
      interrupts();

      if (ready) {
        float speedOfSound = (331.4f + 0.6f * temperature) / 10000.0f;
        calibrated_distance = 1.00338f * widthUs * speedOfSound / 2.0f - 0.2136f;
        distanceValid = widthUs > 0 && widthUs <= ECHO_TIMEOUT_US &&
                        calibrated_distance > 0.0f;
        sonarState = SONAR_IDLE;
      } else if ((uint32_t)(nowUs - sonarTimestampUs) >= ECHO_TIMEOUT_US) {
        noInterrupts();
        captureEcho = false;
        echoReady = false;
        interrupts();
        distanceValid = false;
        sonarState = SONAR_IDLE;
      }
      break;
    }
  }
}

// =========================
// Environment measurement
// =========================

void updateEnvironment(uint32_t nowMs) {
  static uint32_t lastMeasurementMs = 0;
  if (sonarState != SONAR_IDLE) return;
  if ((uint32_t)(nowMs - lastMeasurementMs) >= 3000) {
    if (dht_sensor.measure(&temperature, &humidity)) {
      lastMeasurementMs = millis();
      environmentValid = true;
    }
  }
}

// =========================
// LCD Display
// =========================

void setLcdRow(uint8_t row, const char *text) {
  memset(lcdTarget[row], ' ', 16);
  size_t length = strlen(text);
  if (length > 16) length = 16;
  memcpy(lcdTarget[row], text, length);
  lcdTarget[row][16] = '\0';
}

void updateLcd(uint32_t nowMs) {
  if (displayChanged || (uint32_t)(nowMs - lastLcdUpdateMs) >= LCD_REFRESH_MS) {
    lastLcdUpdateMs = nowMs;
    displayChanged = false;
    char first[24], second[24], line[64];

    switch (displayMode) {
      case 0:
        if (environmentValid) {
          dtostrf(temperature, 4, 1, first);
          snprintf(line, sizeof(line), "Temp: %s C", first);
          setLcdRow(0, line);
          dtostrf(humidity, 4, 1, first);
          snprintf(line, sizeof(line), "Humidity: %s%%", first);
          setLcdRow(1, line);
        } else {
          setLcdRow(0, "Temp: -- C");
          setLcdRow(1, "Humidity: -- %");
        }
        break;

      case 1:
        setLcdRow(0, "Distance:");
        if (distanceValid) {
          dtostrf(calibrated_distance, 6, 1, first);
          snprintf(line, sizeof(line), "%s cm", first);
          setLcdRow(1, line);
        } else {
          setLcdRow(1, "No echo");
        }
        break;

      case 2:
        // Pág 2: Consumo actual (A), tensión (V) y acumulado (mAh)
        dtostrf(average_current, 5, 2, first);
        dtostrf(battery_voltage, 4, 1, second);
        snprintf(line, sizeof(line), "I:%sA V:%sV", first, second);
        setLcdRow(0, line);
        dtostrf(consumed_mAh, 6, 1, first);
        snprintf(line, sizeof(line), "Used:%smAh", first);
        setLcdRow(1, line);
        break;

      case 3:
        // Pág 3: Pitch y roll en grados (con signo, respecto a la calibración)
        if (imuOk) {
          dtostrf(pitchSigned, 6, 1, first);
          snprintf(line, sizeof(line), "Pitch:%s deg", first);
          setLcdRow(0, line);
          dtostrf(rollSigned, 6, 1, first);
          snprintf(line, sizeof(line), "Roll: %s deg", first);
          setLcdRow(1, line);
        } else {
          setLcdRow(0, "Pitch: -- deg");
          setLcdRow(1, "Roll:  -- deg");
        }
        break;
    }
  }

  for (uint8_t index = 0; index < 32; ++index) {
    uint8_t row = index / 16, column = index % 16;
    if (lcdWritten[row][column] != lcdTarget[row][column]) {
      lcd.setCursor(column, row);
      lcd.write((uint8_t)lcdTarget[row][column]);
      lcdWritten[row][column] = lcdTarget[row][column];
      break;
    }
  }
}

// =========================
// Serial Output
// =========================

void updateSerial(uint32_t nowMs) {
  static uint32_t lastUpdateMs = 0;
  static char buffer[256];
  static size_t length = 0, position = 0;

  if (position == length && (uint32_t)(nowMs - lastUpdateMs) >= 500) {
    lastUpdateMs = nowMs;
    char currentText[24], voltageText[24], chargeText[24], distanceText[24];
    char pitchText[24], rollText[24];
    dtostrf(pitch, 1, 1, pitchText);
    dtostrf(roll, 1, 1, rollText);
    dtostrf(average_current, 1, 3, currentText);
    dtostrf(battery_voltage, 1, 2, voltageText);
    dtostrf(consumed_mAh, 1, 2, chargeText);
    if (distanceValid) dtostrf(calibrated_distance, 1, 1, distanceText);
    else strcpy(distanceText, "NA");
    snprintf(buffer, sizeof(buffer),
             "X:%d Y:%d PWM:%d Motor:%d Safety:%d I:%sA V:%sV Used:%smAh D:%s IMU:%d E:%u Cal:%d P:%s R:%s Servo:%d Loop:%lu/%luus\n",
             x_value, y_value, pwmValue, (int)motor_state, (int)safety_switch,
             currentText, voltageText, chargeText, distanceText,
             (int)imuOk, (unsigned int)imuErrors, digitalRead(CalibrationButton_PIN), pitchText, rollText, servoCommand,
             (unsigned long)loopTimeUs, (unsigned long)loopTimeMaxUs);
    length = strlen(buffer);
    position = 0;
  }

  int space = Serial.availableForWrite();
  if (space > 0 && position < length) {
    size_t count = length - position;
    if (count > (size_t)space) count = space;
    position += Serial.write((const uint8_t *)buffer + position, count);
  }
}

// =========================
// Indicador LED de batería LiPo
// =========================

void ledColor(uint32_t nowMs) {
  static uint32_t lastToggleMs = 0;
  static bool toggle = false;

  if (nowMs - lastToggleMs >= 500) {
    lastToggleMs = nowMs;
    toggle = !toggle;
  }

  bool red = false;
  bool green = false;
  bool blue = false;

  if (battery_voltage > 4.2f) {
    red = toggle;
    green = !toggle;
  } else if (battery_voltage >= 3.9f) {
    green = true;
  } else if (battery_voltage >= 3.7f) {
    red = true;
    green = true;
  } else if (battery_voltage >= 3.5f) {
    red = true;
  } else {
    red = true;
    green = toggle;
  }

  digitalWrite(RED_LED_PIN, red ? HIGH : LOW);
  digitalWrite(GREEN_LED_PIN, green ? HIGH : LOW);
  digitalWrite(BLUE_LED_PIN, blue ? HIGH : LOW);
}

// =========================
// GY-521 (MPU-6050) Module
// =========================

// Normaliza un ángulo a [-180, 180] para que la calibración no falle cerca de ±180°
float wrapAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// Comprobación rápida de presencia en el bus I2C (no bloquea gracias al timeout de Wire).
bool imuPresent() {
  Wire.beginTransmission(MPU6050_ADDR);
  return Wire.endTransmission() == 0;
}

// Inicia la IMU con la librería de Jarzebski: begin() comprueba WHO_AM_I y despierta el sensor.
// Antes se comprueba que el sensor responde, porque begin() espera datos sin límite de tiempo.
bool initImu() {
  Wire.begin();
  if (!imuPresent()) return false;
  bool ok = mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
#ifdef WIRE_HAS_TIMEOUT
  // Si el bus se queda colgado (cable suelto, ruido...), Wire aborta y reinicia el hardware I2C
  // en lugar de esperar para siempre. begin() llama a Wire.begin(), así que se fija después.
  Wire.setWireTimeout(3000, true);
#endif
  return ok;
}

// Lectura del acelerómetro con límite de tiempo. Equivale a readNormalizeAccel() del ejemplo,
// pero sin la espera infinita "while (Wire.available() < 6)" de la librería, que dejaba el
// programa congelado si el sensor dejaba de responder a mitad de lectura.
// Con rango ±2 g la sensibilidad es 16384 LSB/g (solo importan las proporciones entre ejes).
bool readAccelBounded(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)6, (uint8_t)true) != 6) return false;

  uint8_t b[6];
  for (uint8_t i = 0; i < 6; ++i) b[i] = Wire.read();
  ax = (int16_t)((b[0] << 8) | b[1]) / 16384.0f;
  ay = (int16_t)((b[2] << 8) | b[3]) / 16384.0f;
  az = (int16_t)((b[4] << 8) | b[5]) / 16384.0f;
  return true;
}

// Lee la IMU y calcula pitch/roll y el incremento de PWM por inclinación.
// NO modifica pwmValue: el acelerador final se compone en computeThrottle().
void ModuleGY521(uint32_t nowMs) {
  static uint32_t lastInitAttemptMs = 0;

  // IMU perdida: sin refuerzo de PWM y se reintenta cada 1 s sin bloquear el programa
  if (!imuOk) {
    incrementPWM = 0.0f;
    if ((uint32_t)(nowMs - lastInitAttemptMs) < 1000) return;
    lastInitAttemptMs = nowMs;
    imuOk = initImu();
    imuFailCount = 0;
    if (!imuOk) return;
  }

  float ax, ay, az;
  // Fallo de lectura, o tres ejes a 0 exactos (imposible con el sensor despierto: se reinició)
  if (!readAccelBounded(ax, ay, az) || (ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    incrementPWM = 0.0f;  // Fallo seguro: no aplicar un refuerzo obsoleto
    ++imuErrors;
    if (++imuFailCount >= 3) imuOk = false;
    return;
  }
  imuFailCount = 0;

  // Calculate Pitch & Roll (mismas fórmulas del ejemplo, en float para no truncar a enteros)
  float rawPitch = -(atan2(ax, sqrt(ay * ay + az * az)) * 180.0) / M_PI;
  float rawRoll  =  (atan2(ay, az) * 180.0) / M_PI;

  // Calibración: UNA sola vez por pulsación (ya no mientras el pin esté en LOW).
  // Fija la orientación actual como 0° de pitch y 0° de roll.
  if (calibrateRequested) {
    pitchOffset = rawPitch;
    rollOffset  = rawRoll;
    calibrateRequested = false;
  }

  pitchSigned = wrapAngle(rawPitch - pitchOffset);
  pitch = fabs(pitchSigned);
  rollSigned = wrapAngle(rawRoll - rollOffset);
  roll  = fabs(rollSigned);

  // Find maximum inclination angle
  maxInclinationValue = max(pitch, roll);

  // PWM boost lineal: 0 a 0° y 255 (acelerador máximo) a 45° o más de pitch/roll
  // (255 / 45° ≈ 5.667 cuentas PWM por grado)
  incrementPWM = 255.0f * min(maxInclinationValue, MAX_TILT_DEG) / MAX_TILT_DEG;
}

// Mueve el servo según el pitch con signo, sin bloquear (nada de delay()).
// Si la IMU se pierde, el servo vuelve al centro.
void updateServo(uint32_t nowMs) {
  static uint32_t lastServoMs = 0;
  if ((uint32_t)(nowMs - lastServoMs) < SERVO_UPDATE_MS) return;
  lastServoMs = nowMs;

  float target = SERVO_CENTER_DEG;
  if (imuOk) {
    float tilt = constrain(pitchSigned, -MAX_TILT_DEG, MAX_TILT_DEG);
    if (SERVO_INVERT) tilt = -tilt;
    target = SERVO_CENTER_DEG + SERVO_GAIN * tilt * (SERVO_MAX_DEG - SERVO_CENTER_DEG) / MAX_TILT_DEG;
    target = constrain(target, (float)SERVO_MIN_DEG, (float)SERVO_MAX_DEG);  // Evita pasarse del rango
  }

  // Filtro paso bajo para evitar temblor por el ruido del acelerómetro
  servoAngle += SERVO_SMOOTHING * (target - servoAngle);
  int command = constrain((int)(servoAngle + 0.5f), SERVO_MIN_DEG, SERVO_MAX_DEG);
  if (command != servoCommand) {  // Solo se escribe si el ángulo cambia
    pitchServo.write(command);
    servoCommand = command;
  }
}

// Compone el acelerador final a partir del joystick, la estabilización,
// la evitación de impactos y el modo de batería baja.
int computeThrottle(int basePwm) {
  // Sin acelerador del piloto (stick en stop) o con el motor parado no hay refuerzos:
  // antes el refuerzo por inclinación/obstáculo arrancaba el motor aunque el stick estuviera en stop.
  if (!motor_state || basePwm <= 0) {
    incrementPWM2 = 0.0f;
    return 0;
  }

  // Evitación de impactos: a <= 15 cm el motor aumenta potencia (no si la batería está baja,
  // para poder aterrizar).
  if (distanceValid && calibrated_distance <= 15.0f && !lowBatteryMode) {
    incrementPWM2 = 0.69f * 255.0f;
  } else {
    incrementPWM2 = 0.0f;
  }

  int pwm = constrain(basePwm + (int)incrementPWM + (int)incrementPWM2, 0, 255);

  // Restricciones de acelerador por nivel de batería y aterrizaje por ultrasonidos
  if (lowBatteryMode) {
    pwm = (int)(pwm * 0.40f);  // Máximo 40% throttle para aterrizar
    if (distanceValid && calibrated_distance <= 5.0f) {
      pwm = 0;  // Parar el motor a <= 5 cm
    }
  }
  return pwm;
}

// =========================
// Setup & Loop
// =========================

void setup() {
  pinMode(SW_pin, INPUT_PULLUP);
  pinMode(lcd_button, INPUT_PULLUP);
  pinMode(CalibrationButton_PIN, INPUT_PULLUP);  // Antes INPUT (flotante)
  pinMode(motorPwmPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  analogWrite(motorPwmPin, 0);
  digitalWrite(TRIG_PIN, LOW);

  // Wake up MPU-6050
  safetyRawState = safetyStableState = digitalRead(SW_pin);
  lcdRawState = lcdStableState = digitalRead(lcd_button);
  delay(50);  // Deja estabilizar el pin antes de leer el estado de reposo
  calRawState = calStableState = calIdleState = digitalRead(CalibrationButton_PIN);
  safety_switch = safetyStableState == LOW;
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoInterrupt, CHANGE);

  Serial.begin(9600);
  lcd.begin(16, 2);
  setLcdRow(0, "");
  setLcdRow(1, "");

  pitchServo.attach(SERVO_PIN);
  pitchServo.write(SERVO_CENTER_DEG);  // Servo al centro mientras arranca la IMU

  // Inicialización de la IMU igual que en el ejemplo de Jarzebski
  Serial.println("Initialize MPU6050");
  while (!initImu()) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  imuOk = true;
}

void loop() {
  uint32_t cycleStartUs = micros();

  updateDistance();
  if (sonarState == TRIGGER_LOW || sonarState == TRIGGER_HIGH) return;

  uint32_t nowMs = millis();
  x_value = analogRead(X_pin);
  y_value = analogRead(Y_pin);

  // 1) Medidas primero: así el control usa datos del ciclo actual
  //    (antes la batería se leía después de aplicar el PWM y arrancaba con 0 V).
  updateBattery(nowMs);
  updateEnvironment(nowMs);
  ModuleGY521(nowMs);
  updateServo(nowMs);

  // 2) Botones
  updateButton(SW_pin, safetyRawState, safetyStableState, safetyLastChangeMs, nowMs);
  safety_switch = safetyStableState == LOW;
  if (calibrationPressed(nowMs)) {
    calibrateRequested = true;
  }
  if (updateButton(lcd_button, lcdRawState, lcdStableState, lcdLastChangeMs, nowMs)) {
    displayMode = (displayMode + 1) % DISPLAY_COUNT;
    displayChanged = true;
  }

  // 3) Estado del motor
  if (safety_switch) motor_state = false;
  else if (x_value > 150 && x_value < 310) motor_state = true;

  // 4) Acelerador: joystick + estabilización + evitación + batería baja
  int basePwm = constrain(map(y_value, 30, 986, 0, 255), 0, 255);
  pwmValue = computeThrottle(basePwm);

  analogWrite(motorPwmPin, pwmValue);  // pwmValue ya es 0 si el motor está parado

  // 5) Aviso sonoro
  static bool buzzerActive = false;
  bool obstacleClose = distanceValid && calibrated_distance < security_distance;
  if (obstacleClose != buzzerActive) {
    if (obstacleClose) tone(buzzerPin, 523);
    else noTone(buzzerPin);
    buzzerActive = obstacleClose;
  }

  ledColor(nowMs);
  updateLcd(nowMs);
  updateSerial(nowMs);

  // Duración de un ciclo completo (sin contar las iteraciones cortas del disparo del sonar)
  loopTimeUs = micros() - cycleStartUs;
  if (loopTimeUs > loopTimeMaxUs) loopTimeMaxUs = loopTimeUs;
}
