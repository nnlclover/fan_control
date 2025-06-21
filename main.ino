#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <TimerOne.h>

// Изменим пины дисплея на более свободные:
const int rs = A0, en = A1, d4 = A2, d5 = A3, d6 = A4, d7 = A5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define DEBUG
#define ONE_WIRE_BUS 2 // Pin for the temperature sensors

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define ALARM_PIN 13

#define FAN1_PWM_PIN 3
#define FAN2_PWM_PIN 5
#define FAN3_PWM_PIN 6

#define FAN1_ROWS_COUNT 11
#define FAN2_ROWS_COUNT 11
#define FAN3_ROWS_COUNT 11

#define FAN1_ALARM_TEMP 95
#define FAN2_ALARM_TEMP 95
#define FAN3_ALARM_TEMP 95

float fan_1_comparison[FAN1_ROWS_COUNT][2] = {
  {0, 0},
  {10, 100},
  {20, 200},
  {30, 300},
  {40, 400},
  {50, 500},
  {60, 600},
  {70, 700},
  {80, 800},
  {90, 900},
  {100, 1000}
};

float fan_2_comparison[FAN2_ROWS_COUNT][2] = {
  {0, 0},
  {10, 100},
  {20, 200},
  {30, 300},
  {40, 400},
  {50, 500},
  {60, 600},
  {70, 700},
  {80, 800},
  {90, 900},
  {100, 1000}
};

float fan_3_comparison[FAN3_ROWS_COUNT][2] = {
  {0, 0},
  {10, 100},
  {20, 200},
  {30, 300},
  {40, 400},
  {50, 500},
  {60, 600},
  {70, 700},
  {80, 800},
  {90, 900},
  {100, 1000}
};

void setup()
{
  #ifdef DEBUG
  Serial.begin(9600);
  #endif //DEBUG
  sensors.begin();

  pinMode(ALARM_PIN, OUTPUT);

  // Initialize TimerOne for 25 kHz PWM
  Timer1.initialize(40); // 40 microseconds for 25 kHz
  Timer1.attachInterrupt(updatePWM); // Attach the interrupt function

  lcd.begin(16, 2);
}

void alarm(bool state)
{
  digitalWrite(ALARM_PIN, state);
}

float findClosestValue(float fan_comparison[][2], int rows_count, float targetTemperature)
{
  float closestTemperature = fan_comparison[0][0];
  float minDifference = abs(targetTemperature - closestTemperature);
  float closestValue = fan_comparison[0][1];

  for (int i = 1; i < rows_count; i++) {
    float currentTemperature = fan_comparison[i][0];
    float currentDifference = abs(targetTemperature - currentTemperature);

    if (currentDifference < minDifference) {
      minDifference = currentDifference;
      closestTemperature = currentTemperature;
      closestValue = fan_comparison[i][1];
    }
  }

  return closestValue;
}

void updatePWM()
{
  static float fan1Speed = 0;
  static float fan2Speed = 0;
  static float fan3Speed = 0;

  // Update PWM values based on the latest temperatures
  analogWrite(FAN1_PWM_PIN, fan1Speed);
  analogWrite(FAN2_PWM_PIN, fan2Speed);
  analogWrite(FAN3_PWM_PIN, fan3Speed);
}

void loop()
{
  sensors.requestTemperatures();

  bool f_alarm = false;
  float temp[3];

  for (int i = 0; i < 3; ++i) {
    temp[i] = sensors.getTempCByIndex(i);
  }

  // Check alarm and set PWM for each fan
  if (temp[0] > FAN1_ALARM_TEMP) f_alarm = true;
  float fan1Speed = findClosestValue(fan_1_comparison, FAN1_ROWS_COUNT, temp[0]);

  if (temp[1] > FAN2_ALARM_TEMP) f_alarm = true;
  float fan2Speed = findClosestValue(fan_2_comparison, FAN2_ROWS_COUNT, temp[1]);

  if (temp[2] > FAN3_ALARM_TEMP) f_alarm = true;
  float fan3Speed = findClosestValue(fan_3_comparison, FAN3_ROWS_COUNT, temp[2]);

  // Update the PWM values for the fans
  Timer1.setPwmDuty(FAN1_PWM_PIN, fan1Speed);
  Timer1.setPwmDuty(FAN2_PWM_PIN, fan2Speed);
  Timer1.setPwmDuty(FAN3_PWM_PIN, fan3Speed);

  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("0:");
  lcd.print(temp[0], 1);
  lcd.print(" 1:");
  lcd.print(temp[1], 1);

  lcd.setCursor(0, 1);
  lcd.print("2:");
  lcd.print(temp[2], 1);
  lcd.print(f_alarm ? " ALARM" : " OK");

  alarm(f_alarm);

  #ifdef DEBUG
  Serial.print("Тревога: ");
  Serial.println(f_alarm);
  for (int i = 0; i < 3; ++i) {
    Serial.print("Температура ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(temp[i]);
  }
  #endif // DEBUG

  delay(1000); // Update every second
}

