#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <TimerOne.h>

//Пины для подключения дисплея.
const int rs = A0, en = A1, d4 = A2, d5 = A3, d6 = A4, d7 = A5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define DEBUG
//Пин для подключения датчиков температуры.
#define ONE_WIRE_BUS 4 

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Пин вывода сигнала тревога.
#define ALARM_PIN 13

//Пины для вывода ШИМ.
#define FAN1_PWM_PIN 9
#define FAN2_PWM_PIN 10
#define FAN3_PWM_PIN 3

//Кол-во записей в таблице с привязкой температуры к скважности.
#define FAN1_ROWS_COUNT 11
#define FAN2_ROWS_COUNT 11
#define FAN3_ROWS_COUNT 11

//Если хоть 1 датчик имеет температуру более, то включаем тревогу.
#define FAN1_ALARM_TEMP 95
#define FAN2_ALARM_TEMP 95
#define FAN3_ALARM_TEMP 95

//Нижний предел, если включена тревога и температура ниже этого предела, то выключаем тревогу.
#define FAN1_ALARM_RESET_TEMP 50
#define FAN2_ALARM_RESET_TEMP 50
#define FAN3_ALARM_RESET_TEMP 50



float fan_1_comparison[FAN1_ROWS_COUNT][2] = {
  {0, 0},
  {10, 0},
  {20, 0},
  {30, 0},
  {40, 40},
  {50, 50},
  {60, 600},
  {70, 700},
  {80, 800},
  {90, 900},
  {100, 1000}
};

float fan_2_comparison[FAN2_ROWS_COUNT][2] = {
  {0, 0},
  {10, 0},
  {20, 0},
  {30, 0},
  {40, 40},
  {50, 50},
  {60, 600},
  {70, 700},
  {80, 800},
  {90, 900},
  {100, 1000}
};

float fan_3_comparison[FAN3_ROWS_COUNT][2] = {
  {0, 0},
  {10, 0},
  {20, 0},
  {30, 0},
  {40, 40},
  {50, 50},
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

  pinMode(FAN1_PWM_PIN, OUTPUT);
  pinMode(FAN2_PWM_PIN, OUTPUT);
  pinMode(FAN3_PWM_PIN, OUTPUT);

  pinMode(ALARM_PIN, OUTPUT);

  // Инициализируем TimerOne на частоте 25 kHz ШИМ
  Timer1.initialize(40); // 40 microseconds for 25 kHz

  lcd.begin(16, 2);

  /*
  TCCR2A = 0;               // Сбрасываем регистр управления таймером 2
TCCR2B = 0;               // Сбрасываем регистр управления таймером 2
TCNT2 = 0;                // Устанавливаем начальное значение счетчика таймера 2
TCCR2A |= (1 << COM2B1);  // Устанавливаем режим non-inverting ШИМ на пине OC2B (пин 10)
TCCR2A |= (1 << WGM21) | (1 << WGM20); // Устанавливаем режим Fast PWM
TCCR2B |= (1 << CS20);    // Устанавливаем предделитель на 1

OCR2A = 639;  
*/


TCCR2A = 0;
TCCR2B = 0;
TCNT2 = 0;

TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // non-inverting PWM, Fast PWM режим 7 (WGM22=1)
TCCR2B |= (1 << WGM22) | (1 << CS11);  // WGM22=1 (режим 7), предделитель = 8 (CS11)

OCR2A = 79; // TOP = 79 -> частота ~25 кГц

}

bool f_alarm = false;


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





void setPWMDuty_D3(uint16_t duty)
{
  if (duty > 1024) duty = 1024;
  Serial.print("duty: ");
  Serial.println((uint32_t)duty * OCR2A / 1024);
  OCR2B = (uint8_t)((uint32_t)duty * OCR2A / 1024);
}




void loop()
{
  sensors.requestTemperatures();

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

  if(f_alarm &&
     temp[0] < FAN1_ALARM_RESET_TEMP &&
     temp[1] < FAN2_ALARM_RESET_TEMP &&
     temp[2] < FAN3_ALARM_RESET_TEMP)
  {
    f_alarm = false;
  }

  
  Timer1.pwm(FAN1_PWM_PIN, fan1Speed);
  Timer1.pwm(FAN2_PWM_PIN, fan2Speed);
  setPWMDuty_D3(fan3Speed);
  //Timer1.pwm(FAN3_PWM_PIN, fan3Speed);

  // Display on LCD
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 500) {  // обновляем каждые 500 мс
    lastUpdate = millis();
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
  }
  
}

