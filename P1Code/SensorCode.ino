//File Contains all the functions related to the IR and ultrasonic sensors

//ULTRASONIC SENSOR DISTANCE

//#ifndef GetSonarDist
//======================================================================================
float GetSonarDist() //HC_SR04_range
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("Ultrasonic Distance NOT found");
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      SerialCom->println("Ultrasonic Distance Out of range");
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;

  return cm;
}
//#endif

//IR SENSOR DISTANCE
//======================================================================================
float getIRDistance (int IRPin) {
  float IRArray[5];
  float dist;
  float IRValue = 0;

  for (int i = 0; i < 5; i++) {
    IRArray[i] = analogRead(IRPin);
  }
  //
  IRValue = (IRArray[0] + IRArray[1] + IRArray[2] + IRArray[3] + IRArray[4]) / 5;
  //IRValue = analogRead(IRPin);

  if (IRPin == irLeftL) {
    dist = pow((3148.6 / IRValue), (1 / 0.789));
  }
  else if (IRPin == irRightL) {
    dist = pow((3365.3 / IRValue), (1 / 0.818));
  }
  else if (IRPin == irLeftS) {
    dist = pow((1926.9 / IRValue), (1 / 0.851));
  }
  else {
    dist = pow((1837.2 / IRValue), (1 / 0.813));
  }

  return Kalman(dist, IRPin);
  //return dist;
}

//RESETTING GYRO
//======================================================================================
void resetGyro(void)
{  
  //Initialise the sensor, find value of voltage when gyro is zero
  int i;
  float sum = 0;
  pinMode(gyroPin, INPUT);
  for (i = 0; i < 100; i++) //Read 100 values of voltage when gyro is at still, to calculate the zero-drift.
  {
    sensorValue = analogRead(gyroPin);
    sum += sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100; //Average the sum as the zero drifting
  currentAngle=0; //Reset the global variable to 0
  gyroAngle=0; //Reset the global variable to 0
  gyroRate=0; //Reset the global variable to 0
}

//GYRO ANGLE
//======================================================================================
float getCurrentAngle()
{

  // convert the 0-1023 signal to 0-5v
  gyroRate = (analogRead(gyroPin) * gyroSupplyVoltage) / 1023;

  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage / 1023 * 5);

  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate / gyroSensitivity;

  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
  {
    // we are running a loop in T. one second will run (1000/T).
    float angleChange = angularVelocity / (1000 / 100);

    //***********************************************************************************************
    currentAngle -= angleChange; //MINUS THE ANGLE, BECAUSE TURNING 90 DEGREES IS ANTICLOCKWISE IN CARTESIAN COORDINATES!
    gyroAngle = (1.0464 * currentAngle) - 0.0664;
  }
  // keep the angle between 0-360
  if (gyroAngle < 0)
  {
    gyroAngle += 360;
  }
  else if (gyroAngle > 359)
  {
    gyroAngle -= 360;
  }
  delay(100);
  return gyroAngle;
}

//======================================================================================
//KALMAN FILTER

//float Mut_prevlist[5] = {0,0,0,0,0}; //Initial value/mean of the 5 sensors
////(in order): sonar, irLeftL, irLeftS, irRightL, irRightS
//float Sigmat_prevlist[5] = {999,999,999,999,999}; //Initial Covariance of the 5 sensors

float Kalman(float zt, int sensorPin) {
  double Mut, Mut_next, Sigmat, Sigmat_next, Kt;
  int i;

  if (sensorPin == irLeftL) 
  {i = 1;}
  else if (sensorPin == irLeftS) 
  {i = 2;}
  else if (sensorPin == irRightL) 
  {i = 3;}
  else if (sensorPin == irRightS) 
  {i = 4;}
  else 
  {i = 0;}

  //Prediction
  Mut = Mut_prevlist[i];
  Sigmat = Sigmat_prevlist[i] + RList[i];

  //Correction
  Kt = Sigmat / (Sigmat + QList[i]);
  Mut_next = Mut + Kt * (zt - Mut); //current mean value
  Sigmat_next = (1 - Kt) * Sigmat; //current covariance

  //Update previous values
  Sigmat_prevlist[i] = Sigmat_next;
  Mut_prevlist[i] = Mut_next;

  return Mut_next;
}
