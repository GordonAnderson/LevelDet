#include "ADC.h"

int   ADCmode     = 5;
// ADC window modes
// 0 = No window mode
// 1 = ADC value > LowerLimit
// 2 = ADC value < UpperLimit
// 3 = ADC value is within the window defined by UpperLimit and LowerLimit
// 4 = ADC value is outside the window defined by UpperLimit and LowerLimit
// 5 = ADC value changed
int   LowerLimit  = 2000;
int   UpperLimit  = 2500;
int   Threshold   = 6;
int   RepeatCount = 6;
int   LastADCval  = 0;
int   adcTare     = 0;
int   ISRcount    = 0;
int   sampleNum   = 0;
int   preScaler   = 4;
int   muxPos      = 6;      // 6 for Trinket pin 4 (A4), 7 for Trinket pin 3 (A3)
void  (*ADCchangeFunc)(bool) = NULL;

// ADC change interrupt call back function
void ADCattachInterrupt(void (*isr)(bool))
{
  ADCchangeFunc = isr;
}

// This ADC ISR fires on every ADC conversion. This routine looks at the window flag 
// and will call the attached interrupt when the window condition changes. A repeat count
// filter is applied to filter false signals.
void ADC_Handler(void)
{
   volatile unsigned int static count  = 0;
   volatile unsigned int static countL = 0;
   int t;
  
   volatile int intflag = ADC->INTFLAG.bit.WINMON;
   ADCsync;
   //ISRcount++;
   LastADCval = ADC->RESULT.reg;
   //serial->println(LastADCval);
   ADCsync;
   //return;
   if(intflag == 1)
   {
      if(++countL >= RepeatCount)
      {
        // If here the limit has been exceeded for 
        // RepeatCount readings in a row
        count = 0;
        if(countL == RepeatCount)
        {
          if(ADCchangeFunc != NULL) ADCchangeFunc(true);
        }
        if(countL > 2*RepeatCount) countL--;
      }    
   }
   else
   {
      if(++count >= RepeatCount)
      {
        // If here the ADC value is within the limit for 
        // RepeatCount readings in a row
        if(ADCmode == 5)
        {
          if((t = LastADCval - Threshold) < 0) t = 0;
          ADCsync;
          ADC->WINLT.reg = t; 
          if((t = LastADCval + Threshold) > MAXADC) t = MAXADC;
          ADCsync;
          ADC->WINUT.reg = t;
        }
        countL = 0;
        if((count == RepeatCount) && (ADCchangeFunc != NULL)) ADCchangeFunc(false);
        if(count > 2*RepeatCount) count--;
      }    
   }
}

// This function enables the ADC change detection system. The variables:
//   ADCmode
//   LowerLimit
//   UpperLimit
// need to be set before calling this function.  
// MUXPOS = 6 for Trinket pin 4 (A4)
// MUXPOS = 7 for Trinket pin 3 (A3)
void ADCchangeDet(void)
{
   pinPeripheral(A4, PIO_ANALOG);
   // Select the ADC channel
   //ADCsync;
   //ADC->AVGCTRL.reg = 0x48;
   ADCsync;
   ADC->CTRLA.bit.ENABLE = 0;
   ADCsync;
   ADC->INPUTCTRL.bit.MUXPOS = muxPos;
   ADCsync;
   // Controls conversion rate, minimum value for 12 bits is 2. 4 = 62,500 sps, 2 = 250,000 sps.
   // This assumes clock is 48MHz. 
   ADC->CTRLB.bit.PRESCALER = preScaler;
   ADCsync;
   ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
   ADCsync;
   ADC->AVGCTRL.bit.SAMPLENUM = sampleNum;
   ADCsync;
   ADC->AVGCTRL.bit.ADJRES = 0; 
   ADCsync;
   ADC->CTRLB.bit.FREERUN = 1;
   ADCsync;
   ADC->CTRLA.bit.ENABLE = 1;
   ADCsync;
   if(ADCmode == 5) ADC->WINCTRL.bit.WINMODE = 3;
   else ADC->WINCTRL.bit.WINMODE = ADCmode;
   ADCsync;
   ADC->INTENSET.bit.RESRDY = 1;
   ADCsync;
   ADC->WINLT.reg = LowerLimit; 
   ADCsync;
   ADC->WINUT.reg = UpperLimit;
   // enable interrupts
   NVIC_EnableIRQ(ADC_IRQn);
   // Start adc
   ADCsync;
   ADC->SWTRIG.bit.START = 1;
}
