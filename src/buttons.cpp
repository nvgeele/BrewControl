#include "buttons.h"
#include "pins.h"

volatile byte button_enc, button_a, button_b = 0;

void setup_buttons() {
  pinMode(BUT_ENC, INPUT_PULLUP);
  pinMode(BUT_A, INPUT_PULLUP);
  pinMode(BUT_B, INPUT_PULLUP);
}

void read_buttons() {
  bool pin_status;
  // delay(3);

  pin_status = digitalRead(BUT_A);
  if(pin_status == LOW && button_a != 128) { button_a = 128; }
  if(pin_status == HIGH && button_a == 128) { button_a = 1; }

  pin_status = digitalRead(BUT_B);
  if(pin_status == LOW && button_b != 128) { button_b = 128; }
  if(pin_status == HIGH && button_b == 128) { button_b = 1; }

  pin_status = digitalRead(BUT_ENC);
  if(pin_status == LOW && button_enc != 128) { button_enc = 128; }
  if(pin_status == HIGH && button_enc == 128) { button_enc = 1; }
}

void clear_buttons() {
  button_a = 0;
  button_b = 0;
  button_enc = 0;
}