#include "pins.h"
#include "buttons.h"

#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

#define LCD_COLS 16
#define LCD_ROWS 2

hd44780_I2Cexp lcd(0x27);

int s_a, s_b, s_c, s_ssr = 0;

void setup_lcd() {
  int status = lcd.begin(LCD_COLS, LCD_ROWS);
  if(status) {
    hd44780::fatalError(-status);
    exit(0);
  }
}

void setup() {
  setup_lcd();
  setup_buttons();

  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  pinMode(MASH_PIN, OUTPUT);
  digitalWrite(MASH_PIN, LOW);
  pinMode(BOIL_PIN, OUTPUT);
  digitalWrite(BOIL_PIN, LOW);
  pinMode(ELEM_PIN, OUTPUT);
  digitalWrite(ELEM_PIN, LOW);

  /*
#define ONE_WIRE_BUS 10
#define ENC_A 2
#define ENC_B 3
*/
}

void loop() {
  read_buttons();
  if(button_a == BUT_PRESSED) {
    s_a = !s_a;
    digitalWrite(MASH_PIN, s_a);
    button_a = BUT_OFF;
  }

  if(button_b == BUT_PRESSED) {
    s_b = !s_b;
    digitalWrite(BOIL_PIN, s_b);
    button_b = BUT_OFF;
  }

  if(button_enc == BUT_PRESSED) {
    s_c = !s_c;
    digitalWrite(ELEM_PIN, s_c);
    button_enc = BUT_OFF;
  }

  if(s_a && s_c && s_b && !s_ssr) {
    s_ssr = !s_ssr;
    digitalWrite(SSR_PIN, HIGH);
  }
  if(s_ssr && !(s_a && s_c && s_b)) {
    s_ssr = !s_ssr;
    digitalWrite(SSR_PIN, LOW);
  }
}