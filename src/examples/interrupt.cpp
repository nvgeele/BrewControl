#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

#define BUTTON_PIN    2
#define LED_PIN       8

const int LCD_COLS = 16;
const int LCD_ROWS = 2;

hd44780_I2Cexp lcd(0x27);

volatile byte state = LOW;
byte pressed = false;
byte enabled = true;

void setup() {
  Serial.begin(9600);
  Serial.println("Serial set up");

  int status;

  status = lcd.begin(LCD_COLS, LCD_ROWS);

  if (status) {
    status = -status;
    hd44780::fatalError(status);
  }

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);
  
//  lcd.print(F("Set LED to 0"));
  lcd.print(F("LED enabled"));

  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  OCR1A = 62500;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);

  sei();
}

ISR(TIMER1_COMPA_vect) {
  state = !state && enabled;
  digitalWrite(LED_PIN, state);
}

void loop() {
  byte r = digitalRead(BUTTON_PIN);
  if (r == LOW && !pressed) {
    enabled = !enabled;
    pressed = true;
    lcd.clear();
    if(enabled)
      lcd.print(F("LED enabled"));
    else
      lcd.print(F("LED disabled"));
  } else if(r == HIGH && pressed) {
    pressed = false;
  }
  delay(50);
  
//  byte r = digitalRead(BUTTON_PIN);
//  if (r == LOW && !pressed) {
//    state = !state;
//    digitalWrite(LED_PIN, state);
//    pressed = true;
//    lcd.moveCursorLeft(); 
//    lcd.print(state);
//  } else if(r == HIGH && pressed) {
//    pressed = false;
//  }
//  delay(50);
  
//  digitalWrite(LED_PIN, HIGH);
//  delayMicroseconds(400);
//  digitalWrite(LED_PIN, LOW);
//  delayMicroseconds(1000-400);
}
