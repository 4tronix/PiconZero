//////////////////////////////////////////////////
//
// Picon Zero Control Software
//
// Gareth Davies 2016
//
//////////////////////////////////////////////////

/* Picon Zero CONTROL
-
- 2 motors driven by H-Bridge: (4 outputs)
- 6 general purpose outputs: Can be LOW, HIGH, PWM, Servo or WS2812B
- 4 general purpose inputs: Can be Analog or Digital
-
- Each I2C packet comprises a Register/Command Pair of bytes

Write Only Registers
--------------------
Register  Name      Type  Values
0   MotorA_Data     Byte  -100 (full reverse) to +100 (full forward)
1   MotorB_Data     Byte  -100 (full reverse) to +100 (full forward)
2   Output0_Config  Byte  0: On/Off, 1: PWM, 2: Servo, (3: WS2812B)
3   Output1_Config  Byte  0: On/Off, 1: PWM, 2: Servo, (3: WS2812B)
4   Output2_Config  Byte  0: On/Off, 1: PWM, 2: Servo, (3: WS2812B)
5   Output3_Config  Byte  0: On/Off, 1: PWM, 2: Servo, (3: WS2812B)
6   Output4_Config  Byte  0: On/Off, 1: PWM, 2: Servo, (3: WS2812B)
7   Output5_Config  Byte  0: On/Off, 1: PWM, 2: Servo, 3: WS2812B
8   Output0_Data    Byte  Data value(s)
9   Output1_Data    Byte  Data value(s)
10  Output2_Data    Byte  Data value(s)
11  Output3_Data    Byte  Data value(s)
12  Output4_Data    Byte  Data value(s)
13  Output5_Data    Byte  Data value(s)
14  Input0_Config   Byte  0: Digital, 1:Analog, 2:DS18B20, 3:DHT11 (NB. 0x80 is Digital input with pullup)
15  Input1_Config   Byte  0: Digital, 1:Analog, 2:DS18B20, 3:DHT11 (NB. 0x80 is Digital input with pullup)
16  Input2_Config   Byte  0: Digital, 1:Analog, 2:DS18B20, 3:DHT11 (NB. 0x80 is Digital input with pullup)
17  Input3_Config   Byte  0: Digital, 1:Analog, 2:DS18B20, 3:DHT11 (NB. 0x80 is Digital input with pullup)
18  Set Brightness  Byte  0..255. Scaled max brightness (default is 40)
19  Update Pixels   Byte  dummy value - forces updating of neopixels
20  Reset           Byte  dummy value - resets all values to initial state

Read Only Registers - These are WORDs
-------------------------------------
Register  Name  Type  Values
0 Revision      Word  Low Byte: Firmware Build, High Byte: PCB Revision
1 Input0_Data   Word  0 or 1 for Digital, 0..1023 for Analog
2 Input1_Data   Word  0 or 1 for Digital, 0..1023 for Analog
3 Input2_Data   Word  0 or 1 for Digital, 0..1023 for Analog
4 Input3_Data   Word  0 or 1 for Digital, 0..1023 for Analog

Data Values for Output Data Registers
--------------------------------------
Mode  Name    Type    Values
0     On/Off  Byte    0 is OFF, 1 is ON
1     PWM     Byte    0 to 100 percentage of ON time
2     Servo   Byte    -100 to + 100 Position in degrees
3     WS2812B 4 Bytes 0:Pixel ID, 1:Red, 2:Green, 3:Blue

*/

/* Rev08: Adds Pullup option for Digital and DS18B20 inputs
*/

#define DEBUG         false
#define BOARD_REV     2   // Board ID for PiconZero
#define FIRMWARE_REV  8   // Firmware Revision

#define MOTORA_DATA   0
#define MOTORB_DATA   1
#define OUTPUT0_CFG   2
#define OUTPUT1_CFG   3
#define OUTPUT2_CFG   4
#define OUTPUT3_CFG   5
#define OUTPUT4_CFG   6
#define OUTPUT5_CFG   7
#define OUTPUT0_DATA  8
#define OUTPUT1_DATA  9
#define OUTPUT2_DATA  10
#define OUTPUT3_DATA  11
#define OUTPUT4_DATA  12
#define OUTPUT5_DATA  13
#define INPUT0_CFG    14
#define INPUT1_CFG    15
#define INPUT2_CFG    16
#define INPUT3_CFG    17
#define SET_BRIGHT    18
#define UPDATE_NOW    19
#define RESET         20


#include <Wire.h>
#include <Servo.h>
#include "FastLED.h"
#include <OneWire.h>

#define I2CADDR     0x22
#define NUMMOTORS   10 // includes 2 for each motor as well as the general purpose output pins
#define NUMOUTPUTS  6 // number of general purpose outputs
#define NUMINPUTS   4 // number of Digital/Analog inputs
#define NUMSERVOS   6 // possible number of servos, all configurable

// Output Config Values
#define CFGONOFF  0
#define CFGPWM    1
#define CFGSERVO  2
#define CFG2812   3

// Input COnfig Values
#define CFGDIG    0
#define CFGDIGPU  0x80
#define CFGANA    1
#define CFG18B20  2
#define CFGDHT11  3

//#define SERVO0    9
//#define SERVO1    10

// WS2812B definitions
#define NUM_LEDS  64
#define DATA_PIN  4
#define BRIGHT    40
#define OUTPUTWS  5 // Output ID that is allowed to use WS2812B mode

CRGB leds[NUM_LEDS];
bool doShow = false;
bool Done2812 = false;

// Constants for Output Pins
#define out0  9
#define out1  10
#define out2  2
#define out3  3
#define out4  4
#define out5  5

// DS18S20 Temperature chip i/o
OneWire ds0(A0);  // on pin A0
OneWire ds1(A1);  // on pin A1
OneWire ds2(A2);  // on pin A2
OneWire ds3(A3);  // on pin A3

// global address data for DS18B20 input devices
byte B20_addr0[8];
byte B20_addr1[8];
byte B20_addr2[8];
byte B20_addr3[8];

// PWM runs from 0 to 100 with 0 being fully OFF and 100 being fully ON
// Outputs are treated as motors (if set to PWM mode)
byte pwm[NUMMOTORS] = {0, 0, 0, 0, 0, 0, 0, 0};  // array of PWM values, one value for each motor/output. Value is the point in the cycle that the PWM pin is switched OFF, with it being switch ON at 0. 100 means always ON: 0 means always OFF
byte motors[NUMMOTORS] = {6, 7, 8, 11, out0, out1, out2, out3, out4, out5}; // array of pins to use for each motor/ouput. Motors have 2 pins, each output has 1 pin
const byte outputs[NUMOUTPUTS] = {out0, out1, out2, out3, out4, out5};
byte inputs[NUMINPUTS] = {A0, A1, A2, A3};
Servo servos[NUMSERVOS];
int inputValues[NUMINPUTS]; // store analog input values (words)
byte outputConfigs[NUMOUTPUTS] = {0, 0, 0, 0, 0, 0};  // 0: On/Off, 1: PWM, 2: Servo, 3: WS2812B
byte inputConfigs[NUMINPUTS] = {0, 0, 0, 0};    // 0: Digital, 1:Analog
byte inputChannel = 0; // selected reading channel
byte pwmcount = 0;
int i;

void setup()
{
  Wire.begin(I2CADDR);            // join i2c bus with defined address
  Wire.onRequest(requestEvent);   // register event
  Wire.onReceive(receiveEvent);   // register wire.write interrupt event
  if (DEBUG)
  {
    Serial.begin(115200);
    Serial.println("Starting...");
    delay(1000);
  }
  for (i = 0; i < NUMMOTORS; i++)
    pinMode (motors[i], OUTPUT);
  FastLED.addLeds<WS2812B, out5, RGB>(leds, NUM_LEDS);  // always have WS2812B enabled on output 5
  FastLED.setBrightness(BRIGHT);  // sets the maximum brightness level. All values are scaled to fit in this range

  resetAll(); // initialise all I/O to safe values
}

// Reset routine should be called at start and end of all programs to ensure that board is set to correct configuration. Python library routines init() and cleanup() will action this
void resetAll()
{
  //disconnect all servos
  for (i=0; i<NUMSERVOS; i++)
    servos[i].detach();
  //clear all WS2812B
  allOff();  
  // set all PWM values to 0 and all outputs to low
  for (i=0; i<NUMMOTORS; i++)
  {
    pwm[i] = 0;
    digitalWrite(motors[i], LOW);
  }
  // set all inputs to Digital
  for (i=0; i<NUMINPUTS; i++)
    inputConfigs[i] = CFGDIG;
  // set all outputs to On/Off
  for (i=0; i<NUMOUTPUTS; i++)
    outputConfigs[i] = CFGONOFF;
}


// The main loop handles the PWM for every motor. Counts from 1..100 and matches the count value with the pwm values. If the same then the signal goes from Low to High
void loop()
{
  //Serial.println("Looping...");
  if (pwmcount == 0)
  {
    for (i=0; i<NUMMOTORS; i++)
      if (pwm[i]>0 && pwm[i]<100)    // PWM values of 0 or 100 means no PWM, so never change this pin
      {
        //Serial.println("High:" + String(motors[i]));
        digitalWrite (motors[i], HIGH);
      }
  }
  else
  {
    for (i=0; i<NUMMOTORS; i++)
      if (pwm[i] == pwmcount)
      {
        //Serial.println("Low:" + String(motors[i]));
        digitalWrite (motors[i], LOW);
      }
  }
  //delay(1);
  delayMicroseconds(10);
  if (++pwmcount > 99)  // as pwmcount never goes over 99, then a value of 100+ will never be changed to LOW so will be on permanently
  {
    pwmcount = 0;
    for (int i=0; i<NUMINPUTS; i++)
    {
      switch (inputConfigs[i])
      {
        case CFGDIG:
        case CFGDIGPU: inputValues[i] = digitalRead(inputs[i]); break;
        case CFGANA: inputValues[i] = analogRead(inputs[i]); break;
        case CFG18B20: startConversion(i); inputValues[i] = getTemp(i); break; // data read is from previous conversion as it can take up to 750ms
        case CFGDHT11: inputValues[i] = getDHT(i); break;
      }
    }
    if (doShow)
    {
      FastLED.show();
      doShow = false;
    }
    if (false)
      Serial.println("pwm0:" + String(pwm[0]) + "  pwm1:" + String(pwm[1]));
  }
}

// This function is called for every data read request. We always return a Word (16-bits). Low byte first.
void requestEvent()
{
  byte outBuf[2];
  if (inputChannel == 0)
  {
    outBuf[0] = BOARD_REV;
    outBuf[1] = FIRMWARE_REV;
  }
  else
  {
    outBuf[0] = inputValues[inputChannel-1]&0xff;
    outBuf[1] = inputValues[inputChannel-1]>>8;
  }
  Wire.write(outBuf, 2);
}


// function that executes whenever data is received from master
void receiveEvent(int count)
{
  //return;
  if (DEBUG)
    Serial.println("Data count:" + String(count));
  if (count == 1) // Read request register
  {
    inputChannel = Wire.read();
    if (DEBUG)
      Serial.println("Channel:" + String(inputChannel));
  }
  else if (count == 2)
  {
    byte regSel = Wire.read();
    byte regVal = Wire.read();
    if (DEBUG)
      Serial.println("Register:" + String(regSel) + "  Value:" + String(regVal));
    switch(regSel)
    {
      case MOTORA_DATA: setMotor(0, regVal); break;
      case MOTORB_DATA: setMotor(1, regVal); break;
      case OUTPUT0_CFG: setOutCfg(0, regVal); break;
      case OUTPUT1_CFG: setOutCfg(1, regVal); break;
      case OUTPUT2_CFG: setOutCfg(2, regVal); break;
      case OUTPUT3_CFG: setOutCfg(3, regVal); break;
      case OUTPUT4_CFG: setOutCfg(4, regVal); break;
      case OUTPUT5_CFG: setOutCfg(5, regVal); break;
      case OUTPUT0_DATA: setOutData(0, regVal); break;
      case OUTPUT1_DATA: setOutData(1, regVal); break;
      case OUTPUT2_DATA: setOutData(2, regVal); break;
      case OUTPUT3_DATA: setOutData(3, regVal); break;
      case OUTPUT4_DATA: setOutData(4, regVal); break;
      case OUTPUT5_DATA: setOutData(5, regVal); break;
      case INPUT0_CFG: setInCfg(0, regVal); break;
      case INPUT1_CFG: setInCfg(1, regVal); break;
      case INPUT2_CFG: setInCfg(2, regVal); break;
      case INPUT3_CFG: setInCfg(3, regVal); break;
      case SET_BRIGHT: FastLED.setBrightness(regVal); break;
      case UPDATE_NOW: doShow = true; break;
      case RESET: resetAll(); break;
    }
  }
  else if (count == 5)
  {
    byte updates = Wire.read();
    byte pixel = Wire.read();
    byte red = Wire.read();
    byte green = Wire.read();
    byte blue = Wire.read();
    //if (DEBUG)
    //  Serial.println("Reg:Val::" + String(regSel) + ":" + String(pixel) + "  Red:" + String(red) + "  Green:" + String(green) + "  Blue:" + String(blue));
    if (outputConfigs[OUTPUTWS] == CFG2812)
    {
      if (pixel < NUM_LEDS)
      {
        leds[pixel].g = red;
        leds[pixel].r = green;
        leds[pixel].b = blue;
        if (updates != 0)
          doShow = true;
      }
      else if (pixel == 100)  // special case meaning ALL pixels
      {
        for (int i=0; i<NUM_LEDS; i++)
        {
          leds[i].g = red;
          leds[i].r = green;
          leds[i].b = blue;
        }
        if (updates != 0)
          doShow = true;
      }
    }
  }
  else // something odd happened. Read all outstanding bytes
  {
    if (DEBUG)
      Serial.println("Odd count:" + String(count));
    for (int i=0; i<count; i++)
      Wire.read();
  }
}

// Function to set the Motor states based on I2C commands received
// Byte received is unsigned, so anything over 128 is actually negative. Convert to signed int first
// Value 0 = OFF (Low-Low)
// Value 100+ = Forward (High, Low) no PWM
// Value -100 = Backward (Low, High) no PWM
// Value 1..99 = Forward (High, Low) with PWM value == value
// Value -1 .. -99 = Reverse (Low, High) with PWM value == -command)
// PWM is applied to the first motor pin for Forward and the second motor pin for Reverse
void setMotor (byte motor, byte value)
{
  int sval = (value>127)?(value-256):value;
  int m2 = motor * 2;
  if (DEBUG)
    Serial.println("M2:" + String(m2) + "  aVal:" + String(sval));
  if (motor > 1)
    return; // Invalid motor number
  if (sval == 0)
  {
    digitalWrite(motors[m2], LOW);
    digitalWrite(motors[m2 + 1], LOW);
    pwm[m2] = pwm[m2 + 1] = 0;  // OFF with no PWM
  }
  else if (sval >= 100)
  {
    digitalWrite(motors[m2], HIGH);
    digitalWrite(motors[m2 + 1], LOW);
    pwm[m2] = pwm[m2 + 1] = 0;  // FORWARD with no PWM
  }
  else if (sval <= -100)
  {
    digitalWrite(motors[m2], LOW);
    digitalWrite(motors[m2 + 1], HIGH);
    pwm[m2] = pwm[m2 + 1] = 0;  // REVERSE with no PWM
  }
  else if (sval > 0)
  {
    digitalWrite (motors[m2], HIGH);
    digitalWrite (motors[m2 + 1], LOW);
    pwm[m2] = sval;  // FORWARD with PWM
    pwm[m2 + 1] = 0;
  }
  else // value sval must be < 0
  {
    digitalWrite (motors[m2], LOW);
    digitalWrite (motors[m2 + 1], HIGH);
    pwm[m2] = 0;
    pwm[m2 + 1] = -sval;  // REVERSE with PWM
  }
}

void setServo(byte servo, byte value)
{
  if(DEBUG)
    Serial.println("Servo: " + String(servo) + "  Value:" + String(value));
  servos[servo].write(value);
}

void setOutCfg(byte outReg, byte value)
{
  if (outputConfigs[outReg] == value)
    return; // don't attach same servo twice, or even set the same value as it currently is
  if (outputConfigs[outReg] != CFGSERVO && value == CFGSERVO)
    servos[outReg].attach(outputs[outReg]);
  else if (outputConfigs[outReg] == CFGSERVO && value != CFGSERVO)
    servos[outReg].detach();
/*  if (value == CFG2812 && (Done2812 || outReg != OUTPUTWS)) // Only Output 5 can be set to WS2812, and only once
    return;
  else if (value == CFG2812 && outReg == OUTPUTWS)
  {
    FastLED.addLeds<WS2812B, out5, RGB>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHT);  // sets the maximum brightness level. All values are scaled to fit in this range
    Done2812 = true;  // ensure we can only have one output for WS2812
  }*/
  outputConfigs[outReg] = value;
}

void setInCfg(byte inReg, byte value)
{
  inputConfigs[inReg] = value;
  if (value == CFGDIGPU)
    pinMode(inputs[inReg], INPUT_PULLUP);
  else
    pinMode(inputs[inReg], INPUT);
  if (value == CFG18B20)
  {
    switch (inReg)
    {
      case 0: ds0.reset_search(); ds0.search(B20_addr0); break;
      case 1: ds1.reset_search(); ds1.search(B20_addr1); break;
      case 2: ds2.reset_search(); ds2.search(B20_addr2); break;
      case 3: ds3.reset_search(); ds3.search(B20_addr3); break;
    }
  }
}

void setOutData(byte outReg, byte value)
{
  switch (outputConfigs[outReg])
  {
    case CFGONOFF:
      pwm[outReg + 4] = 0;
      if(value == 0)
        digitalWrite(outputs[outReg], LOW);
      else
        digitalWrite(outputs[outReg], HIGH);
      break;
    case CFGPWM:
      if (value == 0)
      {
        pwm[outReg + 4] = 0;
        digitalWrite(outputs[outReg], LOW);
      }
      else if (value >= 100)
      {
        pwm[outReg + 4] = 0;
        digitalWrite(outputs[outReg], HIGH);
      }
      else
      {
        pwm[outReg + 4] = min(value, 99);
        digitalWrite(outputs[outReg], LOW); // not strictly necessary as PWM will kick in eventually
      }
      break;
    case CFGSERVO:
      servos[outReg].write(value);
      break;
    case CFG2812:
      // do nothing as this shouldn't arise. 5 bytes needed to address pixels.
      break;
  }
}

// Turns all the LEDs to OFF
void allOff()
{
  for (int i=0; i<NUM_LEDS; i++)
    leds[i] = 0;
  FastLED.show();
}

// Sets all the LEDs to the same colour
void setAll(int red, int green, int blue)
{
  for (int i=0; i<NUM_LEDS; i++)
  {
    leds[i].g = red;
    leds[i].r = green;
    leds[i].b = blue;
  }
  FastLED.show();
}

// DS18B20 Temperature Sensor Reading
int getTemp(int index)
{
  byte data[12];
  switch (index)
  {
    case 0:
      ds0.reset();
      ds0.select(B20_addr0);
      ds0.write(0xBE);         // Read Scratchpad
      for ( i = 0; i < 9; i++)
        data[i] = ds0.read();
      break;
    case 1:
      ds1.reset();
      ds1.select(B20_addr1);
      ds1.write(0xBE);         // Read Scratchpad
      for ( i = 0; i < 9; i++)
        data[i] = ds1.read();
      break;
    case 2:
      ds2.reset();
      ds2.select(B20_addr2);
      ds2.write(0xBE);         // Read Scratchpad
      for ( i = 0; i < 9; i++)
        data[i] = ds2.read();
      break;
    case 3:
      ds3.reset();
      ds3.select(B20_addr3);
      ds3.write(0xBE);         // Read Scratchpad
      for ( i = 0; i < 9; i++)
        data[i] = ds3.read();
      break;
  }
  return data[1]*256 + data[0];
}

void startConversion(int index)
{
  switch (index)
  {
    case 0: ds0.reset(); ds0.select(B20_addr0); ds0.write(0x44,0); break;
    case 1: ds1.reset(); ds1.select(B20_addr1); ds1.write(0x44,0); break;
    case 2: ds2.reset(); ds2.select(B20_addr2); ds2.write(0x44,0); break;
    case 3: ds3.reset(); ds3.select(B20_addr3); ds3.write(0x44,0); break;
  }
}

// DHT11 Humidity & Temp Sensor Reading
int getDHT(int index)
{
  
}

