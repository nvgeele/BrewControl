/* Example of use:
   #define DEBUG          // <-----<<<< this line must appear before the include line
   #include <debug.h>
*/

//If you comment the line:  #define DEBUG
//The Macro lines are defined as blank, thus would be ignored by the compiler.
//If the line is NOT commented, these macros will be included in the sketch.
// examples:
// This  converts to         >>>>----->         This OR a Blank Line.
//--------------------------------------------------------------------------------------------
// DPRINTLN("Testing123");   >>>>----->     Serial.println("Testing123");
// DPRINTLN(0xC0FFEEul,DEC); >>>>----->     Serial.println(0xC0FFEEul,DEC);
// DPRINTLN(12648430ul,HEX); >>>------>     Serial.println(12648430ul,HEX);
// DPRINTLNF("This text came from flash");  Serial.println(F("This text came from flash"));
// DPRINT(myVariable);       >>>>----->     Serial.print(myVariable);
// DELAY(100);               >>>>----->     delay(100);
// SERIALBEGIN(9600);        >>>>----->     Serial.begin(9600);
// PINMODE(13,OUTPUT);       >>>>----->     pinMode(13,OUTPUT);
// TOGGLEd13;                >>>>----->     PINB = 0x20;  // D13 Toggle,for UNO ONLY
 
#ifdef DEBUG
//#define DPRINT(args...)  Serial.print(args)             //OR use the following syntax:
#define SERIALBEGIN(...)   Serial.begin(__VA_ARGS__)
#define DPRINT(...)        Serial.print(__VA_ARGS__)
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#define DRINTF(...)        Serial.print(F(__VA_ARGS__))
#define DPRINTLNF(...)     Serial.println(F(__VA_ARGS__)) //Printing text using the F macro
#define DELAY(...)         delay(__VA_ARGS__)
#define PINMODE(...)       pinMode(__VA_ARGS__)
#define TOGGLEd13          PINB = 0x20                    //For the UNO only
 
#else

#define SERIALBEGIN(...)   //blank line
#define DPRINT(...)        //blank line
#define DPRINTLN(...)      //blank line
#define DPRINTF(...)       //blank line
#define DPRINTLNF(...)     //blank line
#define DELAY(...)         //blank line
#define PINMODE(...)       //blank line
#define TOGGLEd13          //blank line
 
#endif
//***************************************************************
      