#ifndef ADC_h
#define ADC_h
#include <Arduino.h>

#define ADCsync while(ADC->STATUS.bit.SYNCBUSY == 1);
#define MAXADC 4095

extern int   ADCmode;
extern int   LowerLimit;
extern int   UpperLimit;
extern int   RepeatCount;
extern int   Threshold;
extern int   LastADCval;
extern int   adcTare;
extern int   sampleNum;
extern int   preScaler;
extern int   muxPos;

void ADCchangeDet(void);
void ADCattachInterrupt(void (*isr)(bool));

#endif
