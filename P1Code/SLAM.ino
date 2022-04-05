//File Contains all the SLAM code for prediction and correction steps

//#include <BasicLinearAlgebra.h>
//using namespace BLA;

//======================================================================================
void prediction(float controlCommands[], float elapsedTime) {

  Matrix<3, 3>Rt;
  Rt.Fill(1); //Uncertainty of odometry (tbh not sure what value should this matrix be filled up with).

  //Setting up variables
  elapsedTime = elapsedTime * 0.001;
  Serial.print("Time in predict: ");
  Serial.println (elapsedTime);

  //predict x y z coordinates
  for (int i = 0; i < 3; i++) {
    currentState(i) =  prevState(i) + controlCommands[i] * elapsedTime;
  }

  //define jacobian Gxt
  Matrix<3, 3> Gxt;
  for (int i = 0; i < 3; i++) {
    Gxt(i, i) =  1;
  }

  //Update the covariance matrixes except landCov
  robotCov = (Gxt * robotCov * ~Gxt) + Rt;
  trCov = Gxt * trCov;
  blCov = ~trCov;
}

//======================================================================================
void correction(float sensorValue, int L[]) {
  if (sensorValue != 9999) { //if sensor value is valid and in range

  }
  Serial.print(atan2(15, 20));
}

//float predictState(int coordinate, float v, float w, double elapsedTime){
//  elapsedTime = elapsedTime/1000.0;
//
//  //Serial.print(A[1][1]);
//
//  float angleChange = w*elapsedTime; //angularvelocity*deltaT
//  float z = this->currentState[2];
//  float newx = this->nextState[0];
//  float newy = this->nextState[1];
//  float newz = this->nextState[2];
//  float x = this->currentState[0];
//  float y = this->currentState[1];
////
////  switch(coordinate) {
////    case 1: //predicting X
////      newx = x + v*elapsedTime;
////      return(newx);
////      break;
////
////    case 2: //predicting Y
////      newy = y + v*elapsedTime;
////      return(newy);
////      break;
////
////    case 3: //predicting Z (theta)
////      newz = z + angleChange;
////      return(newz);
////      break;
////  }
//}
