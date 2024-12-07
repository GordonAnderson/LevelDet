#ifndef LevelDet_h
#define LevelDet_h
#include "Hardware.h"

#define FILTER   0.1

#define SIGNATURE  0xAA55A5A5

#define MAXTABLE 5

#define ESC   27
#define ENQ   5

// TWI commands and constants
#define TWI_SET_ADCMODE     0x01      // Set ADC mode, 0 thru 5, byte
#define TWI_SET_LL          0x02      // Set LowerLimit, in volts, float
#define TWI_SET_UL          0x03      // Set UpperLimit, in volts, float
#define TWI_SET_THRES       0x04      // Set ADC threshold for detection filter, ADC counts, byte
#define TWI_SET_ZP          0x05      // Set the zero point in volts, 0 to 3, float
#define TWI_SET_TMODE       0x06      // Set the Table mode, 0= off, 1 = use raw ADC counts, 2 = use calibrated ADC value

#define TWI_SERIAL          0x27      // This command enables the TWI port to process serial commands
#define TWI_CMD             0x7F      // This command sends a ascii string to the serial command processor
                                      // and returns the response through the TWI interface

#define TWI_READ_ADCRAW     0x81      // Returns ADC raw counts, word
#define TWI_READ_AVALIBLE   0x82      // Returns the number of bytes avalible in output buffer, 16 bit unsigned int
#define TWI_READ_ADC        0x83      // Returns ADC voltage, in volts, float
#define TWI_LOOKUP          0x84      // Use current ADC value to lookup response in the lookup table

// This must match the structure in the MIPS host application
typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the board name, "FAIMSFB"
  int8_t        Rev;                    // Holds the board revision number
  int           TWIadd;                 // Interface address
  // Level detection settings
  int           ADCmode;
  // ADC window modes
  // 0 = No window mode
  // 1 = ADC value > LowerLimit
  // 2 = ADC value < UpperLimit
  // 3 = ADC value is within the window defined by UpperLimit and LowerLimit
  // 4 = ADC value is outside the window defined by UpperLimit and LowerLimit
  // 5 = ADC value changed
  int           LowerLimit;
  int           UpperLimit;
  int           Threshold;
  int           RepeatCount;
  int           sampleNum;
  int           preScaler;
  int           muxPos;
  // ADC/DAC calibration parameters
  ADCchan       Monitor;
  DACchan       ZeroPoint;
  float         ZPV;
  // Lookup table parameters
  int           Tmode;                  // 0=Off, 1 = use ADC counts, 2 = use Calibrated results
  float         ValueList[MAXTABLE];    // ADC value that will be looked up
  float         LookupList[MAXTABLE];   // Translated value
  //
  unsigned int  Signature;              // Must be 0xAA55A5A5 for valid data
} LevelDet;

extern bool MonitorFlag;

// Prototypes...
void ProcessSerial(bool scan = true);
void TareADC(void);
void ReadADC(void);
void SetUpper(char *fval);
void GetUpper(void);
void SetLower(char *fval);
void GetLower(void);
void SetZeroPoint(char *fval);
void SetTableEntry(void);
void GetTableEntry(int index);
void Lookup(void);

#endif
