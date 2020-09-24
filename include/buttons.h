#include <Arduino.h>

#define BUT_PRESSED 1
#define BUT_OFF 0

extern volatile byte button_enc, button_a, button_b;

void setup_buttons();
void read_buttons();
void clear_buttons();