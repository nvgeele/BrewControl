#include "main.h"
#include "pins.h"

#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#define DEBUG
#include "debug.h"

#define LCD_COLS 16
#define LCD_ROWS 2

#define SP_ADDR 0
#define KP_ADDR 8
#define KI_ADDR 16
#define KD_ADDR 24

#define MENU_MAX 3

hd44780_I2Cexp lcd(0x27);

OneWire one_wire(ONE_WIRE_BUS);
DallasTemperature sensors(&one_wire);
DeviceAddress temp_sensor;

volatile signed short rotary_direction = 0;
volatile boolean rotary_change = false;

volatile byte button_enc, button_a, button_b = 0;

enum operatingState { OFF, PRE_MASH, MASH, PRE_BOIL, BOIL, AUTOTUNE };
operatingState op_state = OFF;

double set_point;
double pid_input;
double pid_output;

double Kp;
double Ki;
double Kd;

int sample_time = 1000;
int window_size = 10000;
unsigned long window_start_time;

volatile long on_time = 0;

PID *pid = NULL;
PID_ATune *autotune = NULL;

void setup_lcd() {
  int status = lcd.begin(LCD_COLS, LCD_ROWS);
  if(status) {
    DPRINTLNF("Something went wrong (LCD)");
    DPRINTLN(-status, DEC);
    hd44780::fatalError(-status);
    exit(0);
  }
}

void setup_sensor() {
  sensors.begin();
  sensors.setWaitForConversion(true);
  if(!sensors.getAddress(temp_sensor, 0)) {
    lcd.setCursor(0, 1);
    lcd.print(F("1st SensorErr"));
    exit(0);
  }
}

void setup() {
  SERIALBEGIN(9600);
  setup_lcd();
  setup_sensor();

  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), rotary_IRQ, CHANGE);

  pinMode(MASH_PIN, OUTPUT);
  digitalWrite(MASH_PIN, LOW);
  pinMode(BOIL_PIN, OUTPUT);
  digitalWrite(BOIL_PIN, LOW);
  pinMode(ELEM_PIN, OUTPUT);
  digitalWrite(ELEM_PIN, LOW);

  pinMode(BUT_ENC, INPUT_PULLUP);
  pinMode(BUT_A, INPUT_PULLUP);
  pinMode(BUT_B, INPUT_PULLUP);

  sensors.setResolution(temp_sensor, 12);

  // TODO: Load PID parameters
  // TODO: Set up timers and interrupts
}

void rotary_IRQ() {
  static unsigned char rotary_state = 0; // current and previous encoder states

  rotary_state <<= 2;  // remember previous state
  rotary_state |= (digitalRead(ENC_A) | (digitalRead(ENC_B) << 1));  // mask in current state
  rotary_state &= 0x0F; // zero upper nibble

  if (rotary_state == 0x03) { // from 00 to 11, increment counter. Also try 0x0C if unreliable
    // incrementing
    rotary_direction = 1;
    rotary_change = true;
  } else if (rotary_state == 0x09) { // from 10 to 01, decrement counter. Also try 0x06 if unreliable
    // decrementing
    rotary_direction = -1;
    rotary_change = true;
  }
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

void loop() {
  switch(op_state) {
    case OFF:
      off();
      break;
    case PRE_MASH:
      prepare_mash();
      break;
    case MASH:
      mash();
      break;
    case PRE_BOIL:
      prepare_boil();
      break;
    case BOIL:
      boil();
      break;
  }
}

void print_off_menu(int selected) {
  lcd.clear();
  lcd.print(F("Select w ENTER"));
  lcd.setCursor(0, 1);
  switch(selected) {
    case 0:
      lcd.print(F("MASH"));
      break;
    case 1:
      lcd.print(F("BOIL"));
      break;
    case 2:
      lcd.print(F("TUNE"));
      break;
    case 3:
      lcd.print(F("AUTOTUNE"));
      break;
  }
}

/** Can select between modes MASH, BOIL, TUNE, and AUTOTUNE */
void off() {
  int selected = 0;

  free_pid(); // Ensure PID is off
  digitalWrite(SSR_PIN, LOW); // Ensure element off
  
  print_off_menu(selected);

  rotary_change = false;

  while(button_enc != 1) {
    read_buttons();

    if(rotary_change) {
      if(rotary_direction == -1) {
        if(selected == 0)
          selected = MENU_MAX;
        else
          selected -= 1;
      } else {
        if(selected == MENU_MAX)
          selected = 0;
        else
          selected += 1;
      }
      print_off_menu(selected);
      rotary_change = false;
    }
  }

  button_enc = 0;
  switch(selected) {
    case 0:
      op_state = PRE_MASH;
      break;
    case 1:
      op_state = PRE_BOIL;
      break;
    case 2:
      // TODO: Tune
      break;
    case 3:
      op_state = AUTOTUNE;
      prepare_autotune();
      break;
  }
}

void auto_tune() {
  if(autotune->Runtime()) {
    finish_autotune();
  }

  // TODO Read sensor, drive output, ...
}

void prepare_autotune() {
  autotune = new PID_ATune(&pid_input, &pid_output);
  autotune->SetNoiseBand((double)1);
  autotune->SetOutputStep((double)500); // TODO reason about this!!!!
  autotune->SetLookbackSec(20);
  autotune->SetControlType(1);
}

void finish_autotune() {
  // TODO: Read and save parameters
  // TODO: Set PID parameters
  free(autotune);
  autotune = NULL;
}

void load_parameters() {
  // TODO
}

void save_paremeters() {
  // TODO
}

void setup_pid() {
  // Ensure correct parameters are read from EEPROM
  load_parameters();
  // If a PID object is in memory, free it
  if(pid != NULL) { free(pid); }
  // Create new PID object and configure
  pid = new PID(&pid_input, &pid_output, &set_point, Kp, Ki, Kd, DIRECT);
  pid->SetOutputLimits(0, window_size);
  pid->SetSampleTime(sample_time);
  pid->SetMode(AUTOMATIC);
}

void free_pid() {
  if(pid == NULL) { return; }
  free(pid);
  pid = NULL;
}

void run_control() {
  // Read current temperature
  sensors.requestTemperatures();
  pid_input = sensors.getTempC(temp_sensor);
  // Run PID algorithm
  pid->Compute();
  // Set output time to PID output
  on_time = pid_output;
}

void ask_confirm(const __FlashStringHelper * message) {
  lcd.clear();
  lcd.print(message);
  lcd.setCursor(0, 1);
  lcd.print(F("cfrm?"));

  while(button_enc != 1) { read_buttons(); }
  button_enc = 0;
}

void prepare_mash() {
  ask_confirm(F("Check Water Lvl"));
  ask_confirm(F("Check Sensor"));
  ask_confirm(F("Contactor ON"));
  ask_confirm(F("Pump ON"));

  // We set a default mash temperature of 64C.
  set_point = 64;

  setup_pid();
  set_sp();

  window_start_time = millis();
  op_state = MASH;
}

/**
 * Modify the SetPoint.
 * Encoder button toggles between 1 and 0.1 increments.
 * Button A confirms.
 * Button B cancels.
 */
void set_sp() {
  lcd.clear();
  lcd.print(F("Temperature?"));
  lcd.setCursor(0, 1);
  lcd.print(set_point);

  clear_buttons();
  rotary_change = false;

  double new_point = set_point;
  float increment = 1.0;

  while(true) {
    read_buttons();

    if(button_enc == 1) {
      button_enc = 0;
      increment = increment == 0.1 ? 1 : 0.1;
    }

    if(button_a == 1) {
      clear_buttons();
      set_point = new_point;
      return;
    }

    if(button_b == 1) {
      clear_buttons();
      return;
    }
    
    if(rotary_change) {
      if(rotary_direction == -1) { // Decrease
        if(new_point > 0)
          new_point -= increment;
      } else { // Increase
        if(new_point < 100)
          new_point += increment;
      }
      rotary_change = false;

      lcd.setCursor(0, 1);
      lcd.print(new_point);
    }

    if(op_state == MASH) {
      run_control();
    }
  }
}

void mash() {
  // TODO: Print status on screen
  // TODO: Read buttons to change modus (maybe have a menu we can go to?)
  // TODO: Interrupt for output
  // TODO: Free PID after mash mode

  lcd.clear();
  lcd.print(F("SP: "));
  lcd.print(set_point, 1);
  lcd.setCursor(0, 1);
  lcd.print(F("PV: "));

  while(true) {
    read_buttons();

    // TODO: Zo doen of anders???
    // Andere optie: aparte tune_sp modus, iets makkelijker voor het scherm.
    // Probleem: we kunnen die dan niet herbruiken om SP te zetten in pre-mash.
    if(button_enc == 1) {
      clear_buttons();
      set_sp();
    }

    run_control();
    lcd.setCursor(4, 1);
    lcd.print(pid_input, 1);

    delay(100);
  }
}

void prepare_boil() {
  // If we want to enter the boil mode from the start menu, we have to do
  // some setup that is usually already done when coming from the mash
  // mode.
  
  // TODO
}

void boil() {
  // TODO
  // TODO: pre-boil arm contactor
}