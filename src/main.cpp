#include "main.h"

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

#define SSR_PIN 7
#define ONE_WIRE_BUS 10
#define ENC_A 2
#define ENC_B 3
#define MASH_PIN 5
#define BOIL_PIN 6
#define ELEM_PIN 7

#define BUT_ENC 4
#define BUT_A 8
#define BUT_B 9

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

enum operatingState { OFF, PRE_MASH, MASH, BOIL, AUTOTUNE };
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
  // TODO review met input_pullup shit
  bool pin_status;
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
  // sensors.requestTemperatures();
  // sensors.getTempC(temp_sensor);

  DPRINTLN(sensors.getResolution(temp_sensor), DEC);
  exit(0);
  // sensors.setResolution(temp_sensor, 12);

  // switch(op_state) {
  //   case OFF:
  //     off();
  //     break;
  //   case PRE_MASH:
  //     prepare_mash();
  //     break;
  //   case MASH:
  //     mash();
  //     break;
  //   case BOIL:
  //     boil();
  //     break;
  // }
}

/** Can select between modes MASH, BOIL, TUNE, and AUTOTUNE */
void off() {
  int selected = 0;

  free_pid(); // Ensure PID is off
  digitalWrite(SSR_PIN, LOW); // Ensure element off

  lcd.print(F("Select w ENTER"));
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
    }

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

  button_enc = 0;
  switch(selected) {
    case 0:
      op_state = PRE_MASH;
      break;
    case 1:
      // TODO
      break;
    case 2:
      // TODO
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

void prepare_mash() {
  lcd.clear();
  lcd.print(F("Check Water Lvl"));
  lcd.setCursor(0, 1);
  lcd.print(F("cfrm?"));

  while(button_enc != 1) { read_buttons(); }
  button_enc = 0;

  lcd.clear();
  lcd.print(F("Check Sensor"));
  lcd.setCursor(0, 1);
  lcd.print(F("cfrm?"));

  while(button_enc != 1) { read_buttons(); }
  button_enc = 0;

  lcd.clear();
  lcd.print(F("Arm Contactor"));
  lcd.setCursor(0, 1);
  lcd.print(F("cfrm?"));

  while(button_enc != 1) { read_buttons(); }
  button_enc = 0;

  setup_pid();
  window_start_time = millis();
  op_state = MASH;
}

void mash() {
  // TODO Print status on screen
  // TODO Read buttons to change modus (maybe have a menu we can go to?)

  run_control();
}

void boil() {
  // TODO
  // TODO: pre-boil arm contactor
}