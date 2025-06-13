#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

constexpr uint8_t VALVE_PIN      = 10;
constexpr uint8_t FLOW_PIN       = 19;
constexpr uint8_t BUTTON_PIN     = 4;
constexpr uint8_t RIGHT_SERVO_PIN= 1;
constexpr uint8_t LEFT_SERVO_PIN = 0;

/* ── LCD ────────────────────────────────────────────────────────*/
constexpr uint8_t LCD_ADDR = 0x27;
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

/* ── dosing parameters ─────────────────────────────────────────*/
constexpr float    PULSES_PER_LITRE = 450.0f;
constexpr uint16_t VOLUMES_ML[]     = { 500, 1000 };
constexpr uint8_t  NUM_VOLUMES      = sizeof(VOLUMES_ML)/sizeof(VOLUMES_ML[0]);
constexpr uint32_t MENU_ROTATE_MS   = 4000;
constexpr uint32_t DONE_DISPLAY_MS  = 3000;

/* ── servo constants ───────────────────────────────────────────*/
constexpr uint16_t MIN_US = 500, MAX_US = 2500;
constexpr uint16_t MOVE_TIME_MS = 500;
Servo rightServo, leftServo;
bool  rightAtHome = true, leftAtHome = true;

/* ── state machine for water flow ──────────────────────────────*/
enum class State : uint8_t { MENU, POURING, DONE };
State state = State::MENU;
uint8_t  menuIdx   = 0;
uint32_t lastMenuSwitch = 0;
volatile uint32_t pulseCount = 0;
uint32_t pulseTarget = 0;
uint32_t doneTimestamp = 0;

/* ── helpers ───────────────────────────────────────────────────*/
void IRAM_ATTR flowISR() { ++pulseCount; }
inline void valveOpen () { digitalWrite(VALVE_PIN, HIGH); }
inline void valveClose() { digitalWrite(VALVE_PIN, LOW);  }

void showMenu()
{
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Select dose:");
  lcd.setCursor(0,1); lcd.print(VOLUMES_ML[menuIdx]); lcd.print(" mL");
}

void startPour(uint16_t ml)
{
  pulseCount  = 0;
  pulseTarget = static_cast<uint32_t>(ml * PULSES_PER_LITRE / 1000.0f);
  valveOpen();

  lcd.clear();
  lcd.print("Dispensing ");
  lcd.print(ml); lcd.print("mL");

  state = State::POURING;
}

void finishPour()
{
  valveClose();
  lcd.clear();
  lcd.print("Done!");
  doneTimestamp = millis();
  state = State::DONE;
}

/* ── Arduino setup ─────────────────────────────────────────────*/
void setup()
{
  pinMode(VALVE_PIN, OUTPUT); valveClose();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, FALLING);

  Wire.begin(8,9);
  lcd.init(); lcd.backlight();
  showMenu();

  rightServo.attach(RIGHT_SERVO_PIN, MIN_US, MAX_US);
  leftServo .attach(LEFT_SERVO_PIN , MIN_US, MAX_US);
  rightServo.write(0);
  leftServo .write(90);
  delay(MOVE_TIME_MS);
  rightServo.detach();
  leftServo.detach();

  Serial.begin(115200);
  delay(1500);
  Serial.println("\nKeys: r = right, l = left");
}

/* ── main loop ─────────────────────────────────────────────────*/
void loop()
{
  uint32_t now = millis();

/*──── Water-dosing state-machine ────────────────────────────*/
  if (state == State::MENU)
  {
    if (now - lastMenuSwitch >= MENU_ROTATE_MS) {
      menuIdx = (menuIdx + 1) % NUM_VOLUMES;
      showMenu();
      lastMenuSwitch = now;
    }
    if (digitalRead(BUTTON_PIN) == LOW) {
      delay(20);
      if (digitalRead(BUTTON_PIN) == LOW) {
        startPour(VOLUMES_ML[menuIdx]);
        while (digitalRead(BUTTON_PIN) == LOW);   // wait release
      }
    }
  }
  else if (state == State::POURING) {
    if (pulseCount >= pulseTarget) finishPour();
  }
  else if (state == State::DONE) {
    if (now - doneTimestamp >= DONE_DISPLAY_MS) {
      state = State::MENU;
      lastMenuSwitch = now;
      showMenu();
    }
  }

/*──── Servo keyboard handler (independent) ─────────────────*/
  if (Serial.available())
  {
    char c = Serial.read() | 0x20;

    if (c == 'r') {
      rightAtHome = !rightAtHome;
      rightServo.attach(RIGHT_SERVO_PIN, MIN_US, MAX_US);
      rightServo.write(rightAtHome ? 0 : 90);
      delay(MOVE_TIME_MS);
      rightServo.detach();

      lcd.clear();
      lcd.setCursor(0,0); lcd.print("ELECTROLYTES");
      lcd.setCursor(0,1); lcd.print("ON THE WAY");
    }
    else if (c == 'l') {
      leftAtHome = !leftAtHome;
      leftServo.attach(LEFT_SERVO_PIN, MIN_US, MAX_US);
      leftServo.write(leftAtHome ? 90 : 0);
      delay(MOVE_TIME_MS);
      leftServo.detach();

      lcd.clear();
      lcd.setCursor(0,0); lcd.print("BCAA");
      lcd.setCursor(0,1); lcd.print("IS ON THE WAY");
    }

    while (Serial.available()) Serial.read();
  }
}