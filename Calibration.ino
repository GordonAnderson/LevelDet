
int AverageADC(int num)
{
  int adc=0;

  for(int j=0;j<num;j++)
  {
     adc += LastADCval;
     delay(1);
  }
  return adc/num;
}

// This function enables calibration of the Monitor input channel.
void CalibrateMonitor(void)
{
  char   *Token;
  String sToken;
  float  val1,val2,m,b;
  int    adc1,adc2;

  serial->println("Calibrate Monitor input, apply adjutable voltage to input.");
  serial->println("Enter values when prompted.");
  // Set input value 1
  serial->print("Enter input value 1: ");
  while((Token = GetToken(true)) == NULL) ProcessSerial(false);
  serial->println(Token);
  sToken = Token;
  val1 = sToken.toFloat();
  // Read ADC value
  adc1 = AverageADC(100);
  while((Token = GetToken(true)) != NULL) ProcessSerial(false);
  
  // Set input value 2
  serial->print("Enter input value 2: ");
  while((Token = GetToken(true)) == NULL) ProcessSerial(false);
  sToken = Token;
  serial->println(Token);
  val2 = sToken.toFloat();
  // Read ADC value
  adc2 = AverageADC(100);
  while((Token = GetToken(true)) != NULL) ProcessSerial(false);
  
  // Calculate calibration parameters and apply
  m = (adc2 - adc1) / (val2-val1);
  b = adc1 - val1 * m;
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  leveldet.Monitor.m = m;
  leveldet.Monitor.b = b;
}

// This function enables calibration of the ZeroPoint output channel.
void CalibrateZeroPoint(void)
{
  char   *Token;
  String sToken;
  float  val1,val2,m,b;
  int    dac1=100,dac2=1000;

  serial->println("Calibrate ZeroPoint output.");
  analogWrite(DAC0, dac1);
  serial->println("Enter values when prompted.");
  // Set input value 1
  serial->print("Enter voltage on C3: ");
  while((Token = GetToken(true)) == NULL) ProcessSerial(false);
  serial->println(Token);
  sToken = Token;
  val1 = sToken.toFloat();
  while((Token = GetToken(true)) != NULL) ProcessSerial(false);
  
  // Set input value 2
  analogWrite(DAC0, dac2);
  serial->print("Enter voltage on C3: ");
  while((Token = GetToken(true)) == NULL) ProcessSerial(false);
  sToken = Token;
  serial->println(Token);
  val2 = sToken.toFloat();
  while((Token = GetToken(true)) != NULL) ProcessSerial(false);
  
  // Calculate calibration parameters and apply
  m = (dac2 - dac1) / (val2-val1);
  b = dac1 - val1 * m;
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  leveldet.ZeroPoint.m = m;
  leveldet.ZeroPoint.b = b;
  analogWrite(DAC0, Value2Counts(leveldet.ZPV, &leveldet.ZeroPoint));
}
