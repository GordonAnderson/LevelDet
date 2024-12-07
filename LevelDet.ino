//
// LevelDet
//
//  This sketch uses the ADC windowing function coupled with the LevelDet hardware to monitor a voltage input and
//  signal MIPS when an event occurs. This sketch is desiged for a Adafruit Trinket M0, this module is used on the
//  LevelDet hardware.
//
//  The LevelDet module is designed to connect to MIPS using the auxliary connector. A hardware line allows this 
//  module to signal MIPS an event has been detected. MIPS interfaces to this module using the TWI interface and can
//  read the detected voltage that the LevelDet module has sensed.
//
//  Default TWI address, 0x32 = 50 base 10.
//
// To do
//  - Add raw ADC to value conversion equation, mx + b linear conversion.
//  - Add look up table capability to the change detection. This will translate the change detected to the closest
//    value in the look up table. When a change is detected and MIPS is signaled then the look up value can be read 
//    from this module if this mode is enabled.
//    Need to add all the necessary TWI commands to support this mode.
//  - Consider this: Make standard serial commands and "pipe" these commands through the TWI to serial
//    command processor logic. In this case MIPS only needs a generic command to direct host commands to this modules
//    TWI interface. For example TWICMD,brd,add,string to module. Any response is sent by MIPS to host. This
//    will allow MIPS to control this module without knowing any of the module details. 32 char max!
//    Use TWI1CMD,add,string for the Wire1 TWI interface.
//
// Revision history:
//
// Version 1.0, April 12, 2020
//  1.) Initial version
// Version 1.1, June 13, 2020
//  1.) Added TWI_CMD to allow processing ascii commands using the command processor
//  2.) Added lookup table capability
// Version 1.2, July 27, 2020
//  1.) DAC0 is 10 bit and calibration routine failed because code assumed 12 bit
// Version 1.3, July 26, 2021
//  1.) Updated to support use in the electrometer hardware. Added commands to allow
//      setting the prescaler and averaging mode as well as tare to define zero.
// Version 1.4, March 19, 2022
//  1.) Added muxPos command to serial processor, 6 or 7 for A4 or A3
//
// Gordon Anderson
//
#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include "SERCOM.h"
#include <Thread.h>
#include <ThreadController.h>
#include <Adafruit_DotStar.h>

#include <Wire.h>
#include <SPI.h>
#include "Hardware.h"
#include "LevelDet.h"
#include "Errors.h"
#include "Serial.h"
#include "ADC.h"
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>
#include <SerialBuffer.h>

const char   Version[] PROGMEM = "LevelDet version 1.4, Mar 19, 2022";
LevelDet     leveldet;
bool         ReturnAvalible = false;
bool         ADCchanged = false;
bool         MonitorFlag = false;

Adafruit_DotStar strip = Adafruit_DotStar(1, INTERNAL_DS_DATA, INTERNAL_DS_CLK, DOTSTAR_BGR);

SerialBuffer sb;

// Reserve a portion of flash memory to store configuration
// Note: the area of flash memory reserved is lost every time
// the sketch is uploaded on the board.
FlashStorage(flash_leveldet, LevelDet);

// ThreadController that will control all threads
ThreadController control = ThreadController();
//Threads
Thread SystemThread = Thread();

LevelDet Rev_1_leveldet = {
                            sizeof(LevelDet),"LevelDet",1,
                            0x32,
                            5,2000,2500,6,6,0,4,6,
                            0,7.71,1856,
                            0,311.74,-0.38,
                            1.5,
                            0,
                            10,20,30,40,50,
                            1,2,3,4,5,
                            //
                            SIGNATURE
                            };

void msTimerIntercept(void)
{

}
extern void (*mySysTickHook)(void);
void (*mySysTickHook)(void) = msTimerIntercept;

// Find the closest entry in the the, must be at least within error, if error is -1 then the closest value is used.
// Returns the index if fould else -1
int FindTableEntry(float ADCval, float error)
{
  int   index = -1;
  int   i;
  
  for(i=0;i<MAXTABLE;i++)
  {
    if((abs(ADCval-leveldet.ValueList[i]) < error) || (error < 0))
    {
      error = abs(ADCval-leveldet.ValueList[i]);
      index = i;
    }
  }
  return(index);
}

// This function is called when the master asks for data.
// Send up to 32 bytes from the sb structure
void requestEventProcessor(void)
{
  int num = sb.available();

  if(ReturnAvalible)
  {
    ReturnAvalible = false;
    Wire.write(num & 0x0FF);
    Wire.write((num >> 8) & 0x0FF);
    return;    
  }
  for (int i = 0; i < num; i++)
  {
    if (i >= 30) break;
    Wire.write(sb.read());
  }  
}

// This function is called when the master asks for data.
// Send up to 32 bytes.
void requestEvent(void)
{
  requestEventProcessor();
}

// Reads a 16 bit value from the TWI interface, return -1 if two bytes
// were not avalibale
int ReadUnsignedWord(void)
{
  int i;

  if (Wire.available() == 0) return -1;
  i = Wire.read();
  if (Wire.available() == 0) return -1;
  i |= Wire.read() << 8;
  return i & 0xFFFF;
}

bool ReadInt(int *i)
{
  if (Wire.available() == 0) return false;
  *i = Wire.read();
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 8;
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 16;
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 24;
  return true;
}

// Reads a 8 bit value from the TWI interface, return -1 if a byte
// was not avalibale
int ReadUnsignedByte(void)
{
  int i;

  if (Wire.available() == 0) return -1;
  i = Wire.read();
  return i & 0xFF;
}

// Reads a 8 bit signed value from the TWI interface, return false if a byte
// was not avalibale or true if ok
bool ReadByte(int8_t *b)
{
  if (Wire.available() == 0) return false;
  *b = Wire.read();
  return true;
}

bool Read16bitInt(int16_t *shortint)
{
  uint8_t *b = (uint8_t *)shortint;

  if (Wire.available() == 0) return false;
  b[0] = Wire.read();
  if (Wire.available() == 0) return false;
  b[1] = Wire.read();
  return true;
}

// Reads a float value from the TWI interface, return false if float
// was not avalibale
bool ReadFloat(float *fval)
{
  int i;
  uint8_t *b;

  b = (uint8_t *)fval;
  for (int j = 0; j < 4; j++)
  {
    if (Wire.available() == 0) return false;
    b[j] = Wire.read();
  }
  return true;
}

void SendByte(byte bval)
{
  sb.write(bval);
}

void SendWord(int ival)
{
  uint16_t *b;

  b = (uint16_t *)&ival;
  // Send the 16 bit word to the ARB module
  sb.write(b[0]);
  sb.write(b[1]);
}

void SendInt24(int ival)
{
  uint8_t *b;

  b = (uint8_t *)&ival;
  // Send the 24 bit word to the ARB module
  sb.write(b[0]);
  sb.write(b[1]);
  sb.write(b[2]);
}

void SendFloat(float fval)
{
  uint8_t *b;

  b = (uint8_t *)&fval;
  // Send the float to the ARB module
  sb.write(b[0]);
  sb.write(b[1]);
  sb.write(b[2]);
  sb.write(b[3]);
}

void SetZeroPoint(float fval)
{
  leveldet.ZPV = fval;
  analogWrite(DAC0, Value2Counts(leveldet.ZPV,&leveldet.ZeroPoint));
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  uint8_t cmd;
  int i, j, off, count, startI, stopI;
  int8_t b;
  int16_t shortint;
  float fval;

  while (Wire.available() != 0)
  {
    cmd = Wire.read();
    if (serial == &sb)
    {
      if (cmd == ESC) serial = &Serial;
      else PutCh(cmd);
    }
    else switch (cmd)
    {
      case TWI_SERIAL:
        serial = &sb;
        break;
      case TWI_CMD:
        // Process command using the serial processor
        sb.clear();
        serial = &sb;
        // Read the ascii string and place in the serial processor ring buffer.
        for(i=0;i<100;i++)
        {
          if(Wire.available() != 0)
          {
            cmd = Wire.read();
            PutCh(cmd);
            if(cmd == '\n') break;
          }
          delayMicroseconds(100);
        }
        // Process any commands fill sb buffer with results
        while (RB_Commands(&RB) > 0) while (ProcessCommand() == 0);
        serial = &Serial;
        break;
      case TWI_SET_ADCMODE:
        if(ReadByte(&b)) if((b>=0)&&(b<=5)) leveldet.ADCmode = b;
        break;
      case TWI_SET_LL:
        if(ReadFloat(&fval)) leveldet.LowerLimit = Value2Counts(fval,&leveldet.Monitor);
        break;
      case TWI_SET_UL:
        if(ReadFloat(&fval)) leveldet.UpperLimit = Value2Counts(fval,&leveldet.Monitor);
        break;
      case TWI_SET_THRES:
        if(ReadByte(&b)) leveldet.Threshold = b;
        break;
      case TWI_SET_ZP:
        if(ReadFloat(&fval)) SetZeroPoint(fval);
        break;
      case TWI_SET_TMODE:
        break;
      // The following commands return data to the master
      case TWI_READ_AVALIBLE:
        // Set flag to return bytes avalible on the next read from TWI
        ReturnAvalible = true;
        break;
      case TWI_READ_ADCRAW:
        SendWord(LastADCval);
        break;
      case TWI_READ_ADC:
        fval = Counts2Value(LastADCval,&leveldet.Monitor);
        SendFloat(fval);
        break;
      case TWI_LOOKUP:
        if(leveldet.Tmode == 1) SendFloat(leveldet.LookupList[FindTableEntry(LastADCval, -1)]);
        if(leveldet.Tmode == 2) SendFloat(leveldet.LookupList[FindTableEntry(Counts2Value(LastADCval,&leveldet.Monitor), -1)]);
        break;
      default:
        break;
    }
  }
}

// This function is called when an ADC change event happens.
// This function will pulse the control line signaling MIPS of the change.
void ADCchangeDetected(bool state)
{
  if(ADCchanged = state)
  {
    digitalWrite(SGNL,HIGH);
    delayMicroseconds(10);
    digitalWrite(SGNL,LOW);  
  }
}

void setup() 
{    
  LoadAltRev();
  pinMode(13,OUTPUT);
  strip.begin();
  strip.setPixelColor(0, 0, 0, 0);
  strip.show();
  // Read the flash config contents and test the signature
  leveldet = flash_leveldet.read();
  if(leveldet.Signature != SIGNATURE) leveldet = Rev_1_leveldet;
  // Init serial communications
  SerialInit();
  analogReadResolution(12);
  analogWriteResolution(12);
  // Setup TWI as slave to communicate with MIPS.
  Wire.begin(leveldet.TWIadd);              // join i2c bus
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  // Configure Threads
  SystemThread.setName((char *)"Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(25);
  // Add threads to the controller
  control.add(&SystemThread);
  // Init Control line
  pinMode(SGNL,OUTPUT);
  digitalWrite(SGNL,LOW);
  // Set the zero point
  analogWriteResolution(12);
  analogWrite(DAC0, Value2Counts(leveldet.ZPV, &leveldet.ZeroPoint));
  // Init the ADC change parameters and enable
  ADCmode = leveldet.ADCmode;
  LowerLimit = leveldet.LowerLimit;
  UpperLimit = leveldet.UpperLimit;
  Threshold = leveldet.Threshold;
  RepeatCount = leveldet.RepeatCount;
  sampleNum = leveldet.sampleNum;
  preScaler = leveldet.preScaler;
  ADCchangeDet();
  ADCattachInterrupt(ADCchangeDetected);
}

// This function is called at 40 Hz
void Update(void)
{
  static int i = 40;

  if(i-- == 0) // Flash the led to let everyone know we are alive!
  {
    i = 40;
    if(digitalRead(13) == LOW) digitalWrite(13,HIGH);
    else digitalWrite(13,LOW);
  }
  // Update the ADC parameters in case there was a change
  LowerLimit = leveldet.LowerLimit;
  UpperLimit = leveldet.UpperLimit;
  Threshold = leveldet.Threshold;
  RepeatCount = leveldet.RepeatCount;
  if(muxPos != leveldet.muxPos)
  {
    muxPos = leveldet.muxPos;
    ADCchangeDet();
  }
  if(preScaler != leveldet.preScaler)
  {
    preScaler = leveldet.preScaler;
    ADCchangeDet();
  }
  if(sampleNum != leveldet.sampleNum)
  {
    sampleNum = leveldet.sampleNum;
    ADCchangeDet();
  }
  // If ADCmode changes then re-init the ADC
  if(ADCmode != leveldet.ADCmode)
  {
    ADCmode = leveldet.ADCmode;
    ADCchangeDet();
  }
}

// This function process all the serial IO and commands
void ProcessSerial(bool scan)
{
  // Put serial received characters in the input ring buffer
  if (Serial.available() > 0)
  {
    PutCh(Serial.read());
  }
  if (!scan) return;
  // If there is a command in the input ring buffer, process it!
  if (RB_Commands(&RB) > 0) while (ProcessCommand() == 0); // Process until flag that there is nothing to do
}

extern int ISRcount;
void loop() 
{
  ProcessSerial();
  control.run();
  if((ADCchanged) && (MonitorFlag))
  {
    ADCchanged = false;
    serial->println(Counts2Value(LastADCval,&leveldet.Monitor));
  }
  //strip.setPixelColor(0, 64, 0, 0); strip.show(); delay(1000); //red
  //strip.setPixelColor(0, 0, 64, 0); strip.show(); delay(1000); //green
  //strip.setPixelColor(0, 0, 0, 64); strip.show(); delay(1000); //blue
  //serial->println(analogRead(A3));
  //ISRcount = 0;
  //delay(1000);
  //serial->println(ISRcount);
}

//
// Host command functions
//

void SaveSettings(void)
{
  leveldet.Signature = SIGNATURE;
  flash_leveldet.write(leveldet);
  SendACK;
}

void RestoreSettings(void)
{
  static LevelDet ld;
  
  // Read the flash config contents and test the signature
  ld = flash_leveldet.read();
  if(ld.Signature == SIGNATURE) leveldet = ld;
  else
  {
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;
  }
  SendACK;  
}

void Software_Reset(void)
{
  NVIC_SystemReset();  
}

void FormatFLASH(void)
{
  flash_leveldet.write(Rev_1_leveldet);  
  SendACK;
}

void ADCreport(bool limit)
{
  if(limit) serial->println("Above limit");
  else serial->println("Below limit");
}

void Debug(int i)
{
  if(i == 1)
  {
    serial->println(ADC->WINLT.reg);
    serial->println(ADC->WINUT.reg);
    return;
  }
  if(i == 0)
  {
     ADCattachInterrupt(ADCreport);
     ADCchangeDet();
  }
  else serial->println(LastADCval);
}

void TareADC(void)
{
  adcTare = LastADCval - leveldet.Monitor.b;
  SendACK;
}

void ReadADC(void)
{
  SendACKonly;
  serial->println(Counts2Value(LastADCval - adcTare,&leveldet.Monitor));
}

void SetUpper(char *fval)
{
  String sToken;

  sToken = fval;
  leveldet.UpperLimit = Value2Counts(sToken.toFloat(),&leveldet.Monitor);
  SendACK;
}

void GetUpper(void)
{
  SendACKonly;
  serial->println(Counts2Value(leveldet.UpperLimit,&leveldet.Monitor));  
}

void SetLower(char *fval)
{
  String sToken;

  sToken = fval;
  leveldet.LowerLimit = Value2Counts(sToken.toFloat(),&leveldet.Monitor);
  SendACK;
}

void GetLower(void)
{
  SendACKonly;
  serial->println(Counts2Value(leveldet.LowerLimit,&leveldet.Monitor));  
}

void SetZeroPoint(char *fval)
{
  String sToken;

  sToken = fval;
  SetZeroPoint(sToken.toFloat());
  SendACK;
}

// Lookup table commands

// This function will set a lookup table entry.
// Three parameters, index,Lookup value, ADC value.
// Maximum of 5 entries in the list
// The user is expected to update all 5 posible entries.
// This function is called with the parameters in the ring buffer
void SetTableEntry(void)
{
   char   *Token;
   String sToken;
   int    ch;
   float  ADCval, Value;

   while(true)
   {
     // Read all the arguments
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     ch = sToken.toInt();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     Value = sToken.toFloat();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     ADCval = sToken.toFloat();
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the channel and exit if error
     if((ch < 0) || (ch > 4)) break;
     leveldet.ValueList[ch] = ADCval;
     leveldet.LookupList[ch] = Value;
     SendACK;
     return;
   }
   // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// Returns a table entry
// index = 0 thru MAXTABLE - 1
void GetTableEntry(int index)
{
   if((index < 0) || (index >= MAXTABLE))
   {
     SetErrorCode(ERR_BADARG);
     SendNAK;
     return;
   }
   SendACKonly;
   // Report the table entry selected, Lookup value, ADC value
   serial->print(leveldet.LookupList[index]);
   serial->print(", ");
   serial->println(leveldet.ValueList[index]);
}

void Lookup(void)
{
  if(leveldet.Tmode == 0) 
  {
    SendACK;
    return;
  }
  SendACKonly;
  if(leveldet.Tmode == 1)
  {
     int i = FindTableEntry(LastADCval, -1);
     serial->println(leveldet.LookupList[i]);    
  }
  if(leveldet.Tmode == 2)
  {
     int i = FindTableEntry(Counts2Value(LastADCval,&leveldet.Monitor), -1);
     serial->println(leveldet.LookupList[i]);
  }  
}
