//volatile int32_t Counter = 1;
//
//void delaySeconds(int TimedDelaySeconds)
//{
//  for (int i = 0; i < TimedDelaySeconds; i++)
//  {
//    delay(1000);
//  }
//}
//
////===============================================================
//
//void flashLED(uint16_t LedNumber, uint16_t TimedDelay)
//{
//  digitalWrite(LedNumber, HIGH);
//  delaySeconds(TimedDelay);
//  digitalWrite(LedNumber, LOW);
//  delaySeconds(TimedDelay);
//}
//
//
////===============================================================
//
//void serialOutputMonitor(float Value1, float Value2, float Value3)
//{
//  String Delimiter = ", ";
//
//  Serial.print(Value1, DEC);
//  Serial.print(Delimiter);
//  Serial.print(Value2, DEC);
//  Serial.print(Delimiter);
//  Serial.println(Value3, DEC);
//}
//
////===============================================================
//
//void serialOutputPlotter(float Value1, float Value2, float Value3)
//{
//  String Delimiter = ", ";
//
//  Serial.print(Value1, DEC);
//  Serial.print(Delimiter);
//  Serial.print(Value2, DEC);
//  Serial.print(Delimiter);
//  Serial.println(Value3, DEC);
//}
////===============================================================
//
//void bluetoothSerialOutputMonitor(float Value1, float Value2, float Value3)
//{
//  String Delimiter = ", ";
//
//  BluetoothSerial.print(Value1, DEC);
//  BluetoothSerial.print(Delimiter);
//  BluetoothSerial.print(Value2, DEC);
//  BluetoothSerial.print(Delimiter);
//  BluetoothSerial.println(Value3, DEC);
//}
//
////===============================================================
//
//void serialOutput(float Value1, float Value2, float Value3)
//{
//  if (OUTPUTMONITOR)
//  {
//    serialOutputMonitor(Value1, Value2, Value3);
//  }
//
//  if (OUTPUTPLOTTER)
//  {
//    serialOutputPlotter(Value1, Value2, Value3);
//  }
//
//  if (OUTPUTBLUETOOTHMONITOR)
//  {
//    bluetoothSerialOutputMonitor(Value1, Value2, Value3);;
//  }
//}
////  serialOutput(Counter, 99, 999);





























volatile int32_t Counter = 1;

void delaySeconds(int TimedDelaySeconds)
{
  for (int i = 0; i < TimedDelaySeconds; i++)
  {
    delay(1000);
  }
}

//===============================================================

void flashLED(uint16_t LedNumber, uint16_t TimedDelay)
{
  digitalWrite(LedNumber, HIGH);
  delaySeconds(TimedDelay);
  digitalWrite(LedNumber, LOW);
  delaySeconds(TimedDelay);
}


//===============================================================

void serialOutputMonitor(int Value1, int Value2, uint16_t Value3)
{
  String Delimiter = ", ";

  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

//===============================================================

void serialOutputPlotter(int Value1, int Value2, int Value3)
{
  String Delimiter = ", ";

  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}
//===============================================================

void bluetoothSerialOutputMonitor(int Value1, int Value2, int Value3)
{
  String Delimiter = ", ";

  BluetoothSerial.print(Value1, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.print(Value2, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.println(Value3, DEC);
}

//===============================================================

void serialOutput(int Value1, int Value2, int Value3)
{
  if (OUTPUTMONITOR)
  {
    serialOutputMonitor(Value1, Value2, Value3);
  }

  if (OUTPUTPLOTTER)
  {
    serialOutputPlotter(Value1, Value2, Value3);
  }

  if (OUTPUTBLUETOOTHMONITOR)
  {
    bluetoothSerialOutputMonitor(Value1, Value2, Value3);;
  }
}
//  serialOutput(Counter, 99, 999);
