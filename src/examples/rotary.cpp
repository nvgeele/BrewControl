#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

#define ROT_B A3        // rotary B
#define ROT_A 3          // rotary A

const int LCD_COLS = 16;
const int LCD_ROWS = 2;

hd44780_I2Cexp lcd(0x27);

volatile byte rotary_counter = 64; // current "position" of rotary encoder (increments CW)
volatile boolean rotary_change = false; // will turn true if rotary_counter has changed

char str_buf[4];
char tmp_buf[4];

// Function declarations
void rotaryIRQ();

void setup() {
  // Set up all the I/O pins. Unused pins are commented out.
  pinMode(ROT_B, INPUT);
  digitalWrite(ROT_B, HIGH); // turn on weak pullup
  pinMode(ROT_A, INPUT);
  digitalWrite(ROT_A, HIGH); // turn on weak pullup
  
  Serial.begin(9600); // Use serial for debugging
  Serial.println("Begin RG Rotary Encoder Testing");

  int status = lcd.begin(LCD_COLS, LCD_ROWS);
  if (status) {
    status = -status;
    hd44780::fatalError(status);
  }

  attachInterrupt(1, rotaryIRQ, CHANGE);
  
  lcd.print(F("Temp: "));
  lcd.print(rotary_counter);
}

void rotaryIRQ() {
  // Process input from the rotary encoder.
  // The rotary "position" is held in rotary_counter, increasing for CW rotation (changes by one per detent).
  // If the position changes, rotary_change will be set true. (You may manually set this to false after handling the change).

  // This function will automatically run when rotary encoder input A transitions in either direction (low to high or high to low)
  // By saving the state of the A and B pins through two interrupts, we'll determine the direction of rotation
  // int rotary_counter will be updated with the new value, and boolean rotary_change will be true if there was a value change
  // Based on concepts from Oleg at circuits@home (http://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros)
  // Unlike Oleg's original code, this code uses only one interrupt and has only two transition states;
  // it has less resolution but needs only one interrupt, is very smooth, and handles switchbounce well.

  static unsigned char rotary_state = 0; // current and previous encoder states

  rotary_state <<= 2;  // remember previous state
  rotary_state |= (digitalRead(ROT_A) | (digitalRead(ROT_B) << 1));  // mask in current state
  rotary_state &= 0x0F; // zero upper nibble

  if (rotary_state == 0x03) { // from 00 to 11, increment counter. Also try 0x0C if unreliable
    if(rotary_counter < 100) {
      rotary_counter++;
      rotary_change = true;
    }
  } else if (rotary_state == 0x09) { // from 10 to 01, decrement counter. Also try 0x06 if unreliable
    if(rotary_counter > 0) {
      rotary_counter--;
      rotary_change = true;
    }
  }
}

void loop() {
  if (rotary_change) {
    sprintf(tmp_buf, "%d", rotary_counter);
    sprintf(str_buf, "%-3s", tmp_buf);
    lcd.setCursor(6, 0);
    lcd.print(str_buf);
  }
  delay(100);
}