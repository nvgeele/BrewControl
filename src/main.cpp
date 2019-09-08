// #include <Arduino.h>

// void setup() {
//   // put your setup code here, to run once:
// }

// void loop() {
//   // put your main code here, to run repeatedly:
// }



//LAST EDITED 22.10.16
//=====================
#include <Arduino.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
// ************************************************
// Pin definitions
// ************************************************

#define RelayPin 7
#define ONE_WIRE_BUS 12
#define enc_a  2
#define enc_b  3
#define enc_c  8
#define BUTTON_SHIFT  11
#define BUTTON_SELECT 9
#define PWM_OUT 10

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Input1;
double Output;

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// DiSplay Variables and constants
// ************************************************
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 

unsigned long lastInput = 0; // last button press

//byte degree[8] = // define the degree symbol 
//{ B00110, B01001, B01001, B00110, B00000,B00000, B00000, B00000 }; 

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO, PWM};
operatingState opState = OFF;

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor, tempSensor1;
volatile byte   last_enc, enc_status, enc_dir, enc_but, button_shift, button_select, pwm_value;
// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
   Serial.begin(9600);

//=================I2C Bus scanner===============
//  Serial.println ();
//  Serial.println ("I2C scanner. Scanning ...");
//  byte count = 0;
//  
//  Wire.begin();
//  for (byte i = 8; i < 120; i++)
//  {
//    Wire.beginTransmission (i);
//    if (Wire.endTransmission () == 0)
//      {
//      Serial.print ("Found address: ");
//      Serial.print (i, DEC);
//      Serial.print (" (0x");
//      Serial.print (i, HEX);
//      Serial.println (")");
//      count++;
//      delay (1);  // maybe unneeded?
//      } // end of good response
//  } // end of for loop
//  Serial.println ("Done.");
//  Serial.print ("Found ");
//  Serial.print (count, DEC);
//  Serial.println (" device(s).");
//===============================================

   // Initialize Relay Control:

  pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
  digitalWrite(RelayPin, LOW);  // make sure it is off to start
  pinMode(enc_a, INPUT_PULLUP); pinMode(enc_b, INPUT_PULLUP); pinMode(enc_c, INPUT_PULLUP);
  pinMode(BUTTON_SHIFT, INPUT_PULLUP); pinMode(BUTTON_SELECT,INPUT_PULLUP);
  last_enc=3;
  enc_status=0x00;enc_dir=0;enc_but=0;button_shift=0;button_select=0;
  attachInterrupt(digitalPinToInterrupt(enc_a), read_enc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_b), read_enc, CHANGE);

  // Initialize LCD DiSplay 

  lcd.begin(16, 2);lcd.clear();lcd.backlight();lcd.setCursor(0, 0); //(Col, Row);
  lcd.print(F("Starting..."));
  //lcd.print("MSG");
//  lcd.setCursor(0, 1);
//  lcd.print(F("Script!"));

   // Start up the DS18B20 One Wire Temperature Sensor

   sensors.begin();
   if (!sensors.getAddress(tempSensor, 0)) 
   {
      lcd.setCursor(0, 1);
      lcd.print(F("1st SensorErr"));
   }

   if (!sensors.getAddress(tempSensor1, 1)) 
   {
      lcd.setCursor(0, 1);
      lcd.print(F("2st SensorErr"));
   }
   
   sensors.setResolution(tempSensor, 12);
   sensors.setResolution(tempSensor1, 12);
   sensors.setWaitForConversion(false);
   Serial.println(tempSensor[7], HEX);
   Serial.println(tempSensor1[7], HEX);

   delay(3000);  // Splash screen

   // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);

  // Run timer2 interrupt every 15 ms 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;

  pinMode(10, OUTPUT);
 // setPWMFrequency(10,1);  //PIN 10 is set to 31kHz or TCCR1B = (TCCR1B & 0b11111000) | 1;
 TCCR1B = TCCR1B & B11111000 | B00000010;
 pwm_value=0;
 analogWrite(PWM_OUT, pwm_value);
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) 
{
  if (opState == OFF)
  {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

 /* ENCODER ROUTINES
 * CW d135       CCW  d75 = 1/1 4/0 18/2 75/3 
 * =2/2 8/0 33/1 135/3 
 * ---------------------------
 *           11       11
 * 10        10     10
 *   00      00   00
 *     01    01 01      
 *       11  11
 */
//BUTTON INTERRUPT
void read_but()
{
  bool pin_status;
  delay(3);
  
  pin_status=digitalRead(enc_c);
  if(pin_status == LOW && enc_but !=128) {enc_but = 128;}
  if(pin_status == HIGH && enc_but ==128){enc_but=1;lastInput=millis();}

  pin_status=digitalRead(BUTTON_SHIFT);
  if(pin_status == LOW && button_shift !=128) {button_shift = 128;}
  if(pin_status == HIGH && button_shift ==128){button_shift=1;lastInput=millis();}

  pin_status=digitalRead(BUTTON_SELECT);
  if(pin_status == LOW && button_select !=128) {button_select = 128;}
  if(pin_status == HIGH && button_select ==128){button_select=1;lastInput=millis();}
}
void read_enc()
{
  //returns 0 if nothing is set
  //returns 1 if A is set but B is not
  byte a=0;
  delayMicroseconds(200);
  if(digitalRead(enc_a) == HIGH ){a=B01;}
  if(digitalRead(enc_b) == HIGH ){a=a+2;} //set the second bit
  if(last_enc!=a)
  {
    lastInput=millis();
    last_enc = a;
    enc_status = enc_status*4;
    enc_status = enc_status + a;
    //Serial.print(enc_status, DEC);Serial.print("/");Serial.print(a, DEC);Serial.print(" ");
    switch(enc_status)
    {
      case 0x4b:enc_dir=1;enc_status=0;/*Serial.print("\n");*/break;  //ccw, enc_dir=1 
      case 0x87:enc_dir=2;enc_status=0;/*Serial.print("\n");*/break;  //cw enc_dir=2
      default : enc_dir =0;break;
    }
  }
}
// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
   // wait for button release before changing state
   //while(enc_but != 1 ) { read_but();}
   //while(ReadButtons() != 0) {}

   lcd.clear();
//   enc_but = 0;

   switch (opState)
   {
   case OFF:
   
      Off();       
      break;
   case SETP:
      Tune_Sp();         
      break;
    case RUN:
      Run();        
      break;
   case TUNE_P:
      TuneP();        
      break;
   case TUNE_I:
      TuneI();         
      break;
   case TUNE_D:
      TuneD();        
      break;
   case PWM:
      setPWM();
      break;   
   }
}

// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Off()
{
   myPID.SetMode(MANUAL);
 //  lcd.setBacklight(0);
   digitalWrite(RelayPin, LOW);  // make sure it is off
   lcd.print(F("POWER"));
   lcd.setCursor(0, 1);
   lcd.print(F("OFF!"));
   pwm_value = 0;
   analogWrite(PWM_OUT, pwm_value);
   
   while(enc_but != 1){read_but();}
   enc_but =0;button_select = 0;button_shift=0;

   lcd.clear();
   lcd.print(F("Check Water Lvl"));
   lcd.setCursor(0,1);
   lcd.print(F("Press SEL 2 cfrm"));
   while(button_select != 1){read_but(); if(enc_but == 1) { enc_but=0;opState=OFF;return;}}
   button_select =0;
   lcd.clear();
   lcd.print(F("Switching Heat"));
   lcd.setCursor(0,1);
   lcd.print(F("Press SHI 2 cfrm"));
   while(button_shift != 1){read_but();if(enc_but == 1) { enc_but=0;opState=OFF;return;}}
   button_shift =0;
   
   // Prepare to transition to the RUN state
   sensors.requestTemperatures(); // Start an asynchronous temperature reading


   //turn the PID on
   myPID.SetMode(AUTOMATIC);
   windowStartTime = millis();
   opState = RUN; // start control
}

// ************************************************
// Setpoint Entry State
// UP/DOWN to change setpoint
// RIGHT for tuning parameters
// LEFT for OFF
// SHIFT for 10x tuning
// ************************************************
void Tune_Sp()
{
   lcd.clear();
   lcd.print("Set Temperature:");

   button_select = 0;button_shift = 0;enc_but=0;
   while(true)
   {
      //buttons = ReadButtons();
      read_but();

      float increment = 1;
      if (digitalRead(enc_c) == LOW )
      {
        increment *= 0.1;
      }
      if (button_shift == 1)
      {
         opState = RUN;
         button_shift =0;
         Serial.println("G2RUN");
         return;
      }
      if (button_select == 1)
      {
         opState = TUNE_P;
         button_select=0;
         return;
      }
      if (enc_dir == 2)
      {
         Setpoint += increment;
         enc_dir = 0;
         delay(50);
      }
      if (enc_dir==1)
      {
         Setpoint -= increment;
         enc_dir = 0;
         delay(50);
      }
    
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Setpoint);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Proportional Tuning State
// UP/DOWN to change Kp
// RIGHT for Ki
// LEFT for setpoint
// SHIFT for 10x tuning
// ************************************************
void TuneP()
{
 //lcd.setBacklight(TEAL);
   lcd.print(F("Set Kp"));

// uint8_t buttons = 0;
   while(true)
   {
      read_but();

      float increment = 1.0;
      if (digitalRead(enc_c) == LOW )
      {
        increment *= 10;
      }
      if (button_shift == 1)
      {
         opState = SETP;
         button_shift = 0;
         return;
      }
      if (button_select ==1 )
      {
         opState = TUNE_I;
         button_select=0;
         return;
      }
      if (enc_dir == 2 )
      {
         Kp += increment;
         enc_dir = 0;
         delay(50);
      }
      if (enc_dir==1)
      {
         Kp -= increment;
         enc_dir = 0;
         delay(50);
      }
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kp);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Integral Tuning State
// UP/DOWN to change Ki
// RIGHT for Kd
// LEFT for Kp
// SHIFT for 10x tuning
// ************************************************
void TuneI()
{
//   lcd.setBacklight(TEAL);
   lcd.print(F("Set Ki"));

//   uint8_t buttons = 0;
   while(true)
   {
      read_but();

      float increment = 0.01;
      if (digitalRead(enc_c) == LOW )
      {
        increment *= 10;
      }
      if (button_shift == 1 )
      {
         opState = TUNE_P;
         button_shift = 0;
         return;
      }
      if (button_select == 1)
      {
         opState = TUNE_D;
         button_select = 0;
         return;
      }
      if (enc_dir == 2)
      {
         Ki += increment;
         enc_dir = 0;
         delay(50);
      }
      if (enc_dir == 1)
      {
         Ki -= increment;
         enc_dir = 0;
         delay(50);
      }
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Ki);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Derivative Tuning State
// UP/DOWN to change Kd
// RIGHT for setpoint
// LEFT for Ki
// SHIFT for 10x tuning
// ************************************************
void TuneD()
{
//   lcd.setBacklight(TEAL);
   lcd.print(F("Set Kd"));

//   uint8_t buttons = 0;
   while(true)
   {
      read_but();
      
      float increment = 0.01;
      if (digitalRead(enc_c) == LOW )
      {
        increment *= 10;
      }
      if (button_shift == 1)
      {
         opState = TUNE_I;
         button_shift = 0;
         return;
      }
      if (button_select == 1)
      {
         opState = PWM;
         button_select=0;
         return;
      }
      if (enc_dir == 2)
      {
         Kd += increment;
         enc_dir=0;
         delay(50);
      }
      if(enc_dir == 1)
      {
         Kd -= increment;
         enc_dir = 0;
         delay(50);
      }
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kd);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// PID COntrol State
// SHIFT and RIGHT for autotune
// RIGHT - Setpoint
// LEFT - OFF
// ************************************************
void Run()
{
   // set up the LCD's number of rows and columns: 
   lcd.clear();
   lcd.print(F("S"));
   lcd.print(Setpoint,1);
   lcd.print(F("C "));

   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   //uint8_t buttons = 0;
   //read_but();

   button_select = 0;button_shift = 0;enc_but=0;

   while(true)
   {
     read_but();
     
      if ( (button_shift == 1 ) 
         && (button_select == 1 ) 
         && (abs(Input - Setpoint) < 0.5))  // Should be at steady-state
      {        
         button_shift = 0; button_select = 0;
         StartAutoTune();
      }
      else if (button_select == 1 )
      {
        opState = SETP;
        button_select = 0;
        return;
      }
      else if (button_shift == 1)
      {
        opState = OFF;
        button_shift = 0;
        return;
      }
      if(enc_dir==1 || enc_dir==2)
      {
        opState = PWM;
        setPWM();
        return;
      }
      
      DoControl();

      lcd.setCursor(7,0);
      setBacklight();
      float pct = map(Output, 0, WindowSize, 0, 1000);
      //lcd.setCursor(10,1);
      //lcd.setCursor(11,0);
      lcd.print((pct/10),1);
      //lcd.print(Output); //???
      lcd.print("%");
      
      lcd.setCursor(0,1);
      lcd.print(">");
      lcd.print(Input);
      lcd.print(F("C  "));
      lcd.print(Input1);
      lcd.print(F("C"));
//      
//      if (tuning)
//      {
//        lcd.print("T");
//      }
//      else
//      {
//        lcd.print(" ");
//      }
      
      // periodically log to serial port in csv format
//      if (millis() - lastLogTime > logInterval)  
//      {
//        Serial.print(Input);
//        Serial.print(",");
//        Serial.println(Output);
//     }

      delay(100);
   }
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    Input = sensors.getTempC(tempSensor);
    Input1= sensors.getTempC(tempSensor1);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
  
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output; 
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(RelayPin,HIGH);
  }
  else
  {
     digitalWrite(RelayPin,LOW);
  }
}

// ************************************************
// Set Backlight based on the state of control
// ************************************************
void setBacklight()
{
   if (tuning)
   {
      lcd.print("T "); // Tuning Mode
   }
   else if (abs(Input - Setpoint) > 1.0)  
   {
      lcd.print("H ");// High Alarm - off by more than 1 degree
   }
   else if (abs(Input - Setpoint) > 0.2)  
   {
      lcd.print("L ");// Low Alarm - off by more than 0.2 degrees
   }
   else
   {
      lcd.print("* "); // Lock on temp! 
   }
}
// ************************************************
// Set PWM
// ***********************************************
void setPWM()
{
   lcd.clear();
   lcd.print(F("Set PWM"));

   while(true)
   {
      read_but();            

      if (button_shift == 1)
      {
         opState = TUNE_D;
         button_shift = 0;
         return;
      }
      if (button_select == 1)
      {
         opState = RUN;
         button_select=0;
         return;
      }
      if(enc_dir==2)
      {
        enc_dir=0;
        pwm_value += 4;
        printPWM();     
      }     
      if(enc_dir==1)
      {
        enc_dir=0;
        pwm_value -= 4;
        printPWM();
      }
  
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
   }
}

void printPWM()
{
      analogWrite(PWM_OUT, pwm_value);
      lcd.setCursor(0,1);
      lcd.print("  %");
      lcd.setCursor(0,1);
      lcd.print((float(pwm_value)/255)*100,0);
}
// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}

// ************************************************
// Check buttons and time-stamp the last press
// ************************************************
//uint8_t ReadButtons()
//{
//  uint8_t buttons = lcd.readButtons();
//  if (buttons != 0)
//  {
//    lastInput = millis();
//  }
//  return buttons;
//}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}