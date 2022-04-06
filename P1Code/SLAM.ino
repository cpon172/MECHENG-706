//PREDICTION STEP

void prediction(velocities v) {

  //Matrix<11> currentState; //x, y, theta, land1x, land1y, land2x, land2y, land3x, land3y, land4x, land4y
  //  Matrix<3, 3>Rt;
  //  Rt.Fill(1); //Uncertainty of odometry (tbh not sure what value should this matrix be filled up with).
  float Rt = 1;

  currentState(2) = getCurrentAngle();//prevState(2) + v[2] * 0.5 * 180 / PI; //updating Z //rad + rads-1 *s = rad -> degrees

  prevState(2) = currentState(2);

  //updating X
  currentState(0) = prevState(0) + (v.x * cos(currentState(2) * PI / 180) + v.y * sin(currentState(2) * PI / 180)) * 0.5;
  prevState(0) = currentState(0); //making prevX into updated X for next call

  //updating Y
  currentState(1) = prevState(1) + (v.y * cos(currentState(2) * PI / 180) - v.x * sin(currentState(2) * PI / 180)) * 0.5;
  prevState(1) = currentState(1);

  serialOutput(currentState(0), currentState(1), currentState(2));

  //define jacobian Gxt
  Matrix<3, 3> Gxt;
  Gxt.Fill(0);
  for (int i = 0; i < 3; i++) {
    Gxt(i, i) =  1;
  }
  Gxt(2, 0) = 0.5 * (v.y * cos(currentState(2) * PI / 180) - v.x * sin(currentState(2) * PI / 180)); //dx
    Gxt(1, 2) = -0.5 * (v.x * cos(currentState(2) * PI / 180) + v.y * sin(currentState(2) * PI / 180)); //dy
  //    Serial << "Prediction Jacobian: " << Gxt << '\n';

  //Update the covariance matrixes except the landmark matrix
  robotCov = (Gxt * robotCov * ~Gxt) + Rt;
  trCov = Gxt * trCov + Rt;
  blCov = ~trCov + Rt;
  serialOutput(currentState(0), currentState(1), currentState(2)); //Print the current states (currentState(2)) is not necessary to print
}

//======================================================================================
//CORRECTION STEP
void correction(int wallNumber) {
  float r = GetSonarDist(); //returns range of wall from robot in cm

  //  float wallLocation(0,0) = r;
  //  wallLocation(0,1) = currentState(2);

  //define jacobian lowHt
  Matrix<4, 4> lowHt;
  lowHt.Fill(0);

  lowHt(0, 0) = -r * r * cos(currentState(2) * PI / 180);
  lowHt(0, 1) = -r * r * sin(currentState(2) * PI / 180);
  lowHt(0, 2) = 0;
  lowHt(0, 3) = r * r * cos(currentState(2) * PI / 180);
  lowHt(0, 4) = r * r * sin(currentState(2) * PI / 180);

  lowHt(1, 0) = r * sin(currentState(2) * PI / 180);
  lowHt(1, 1) = -r * cos(currentState(2) * PI / 180);
  lowHt(1, 2) = -1 * pow(r, 2);
  lowHt(1, 3) = -r * sin(currentState(2) * PI / 180);
  lowHt(1, 4) = r * cos(currentState(2) * PI / 180);

  lowHt = lowHt * (1 / (r * r));

  switch (wallNumber) {
    case 1: ; //wall 1


    case 2: ; //wall 2

    case 3: ; //wall 3
  }
}

//======================================================================================

//ISR to update coordinates
ISR(TIMER2_COMPA_vect) {
  if (isrCount == 31) { //When 31 ISRs occur, ~0.5 seconds have passed by
    prediction(v);
    //correction(wallNumber);
    isrCount = 0; //Reset count
  }
  isrCount++; //update count
}

//======================================================================================

velocities setV(float vx, float vy, float wz) {
  v.x = vx;
  v.y = vy;
  v.z = wz;
  return v;
}
