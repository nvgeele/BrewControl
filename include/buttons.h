#include <Arduino.h>

extern volatile byte button_enc, button_a, button_b;

void setup_buttons();
void read_buttons();
void clear_buttons();