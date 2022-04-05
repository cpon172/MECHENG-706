//File Contains all the functions related to the motor movement commands
//======================================================================================
void initMoveToWall(float distanceToWall) {
  Serial.println("Entered Intial movement to wall function");
  do {
    moveRobot(0, 0.44, 0);
    uSensor = GetSonarDist();
  } while (uSensor > distanceToWall);
  moveRobot(0, 0, 0);//Stop movement
  delay(1000);
}

//======================================================================================
void moveToCorner(float distanceToWall) {
  Serial.println("Entered move to corner function");

  do {
    moveRobot(0, 0.44, 0);
    uSensor = GetSonarDist();

    if ((getIRDistance(irLeftS) < 12) || (getIRDistance(irLeftL) < 12)) {
      moveRobot(0.2, 0.2, 0);//Move robot forwards + right
      delay(50);
    }
    if ((getIRDistance(irLeftS) > 12) || (getIRDistance(irLeftL) > 12)) {
      moveRobot(-0.2, 0.2, 0);//Move robot forwards + left
      delay(50);
    }
  } while (uSensor > distanceToWall);//Stop when ultrasonic sensor has detected that distance wall has been reached.
  moveRobot(0, 0, 0);
  delay(1000);
}

//======================================================================================
void rotateRobot(float angle, bool angleDirection) {//update function to use both gyro and ir sensors
  Serial.println("Entered rotate robot function");

  float presentAngle = 0; //Reset current angle

  resetGyro();


  if (angleDirection == 1) {         //Rotating CW
    Serial.println("Rotate Clockwise");
    do {
      moveRobot(0, 0, 1);
      //get current angle and print
      presentAngle = getCurrentAngle();
      Serial.print("This is the currentAngle: ");
      Serial.println(presentAngle);
      Serial.print("This is the Angle we want to reach: ");
      Serial.println(angle);
    } while ( (presentAngle < (angle * 0.93)) || (presentAngle == 0) );
  }
  else if (angleDirection == 0) {   //Rotating CCW
    Serial.println("Rotate counterclockwise");
    do {
      moveRobot(0, 0, -1);
      //get current angle and print
      presentAngle = getCurrentAngle();
      Serial.print("This is the currentAngle: ");
      Serial.println(presentAngle);
      Serial.print("This is the Angle we want to reach: ");
      Serial.println(angle);
    } while (((presentAngle) > ((360 - angle) * 1.07)) || (presentAngle == 0));
  }

  moveRobot(0, 0, 0);
  delay(1000);
}

//======================================================================================
void alignWall(bool senseDirection, float DistanceToWall) {
  if (senseDirection) {
    do {
      irRightLVal = getIRDistance(irRightL);
      irRightSVal = getIRDistance(irRightS);
      if (irRightLVal >= irRightSVal) {
        moveRobot(0, 0, -0.5); //Back right is too far away, rotate left
      }
      else if (irRightLVal <= irRightSVal) {
        moveRobot(0, 0, 0.5); //Front Left is too far away, rotate left
      }
      else if ((irRightLVal && irRightSVal) < DistanceToWall) { //Distance too far from wall, moving closer
        moveRobot(1, 0, 0);
      }
      else { //Distance too close to wall, moving closer
        moveRobot(-1, 0, 0);
      }
      delay(50);
    } while ((abs(irRightLVal - irRightSVal) > 1) && (irRightLVal <= DistanceToWall) && (irRightSVal <= DistanceToWall));

  }
  else {
    Serial.println("CHECKING LEFT WALL");
    int counter = 0;
    do {

      irLeftLVal = getIRDistance(irLeftL);
      irLeftSVal = getIRDistance(irLeftS);
      Serial.print("Maximum Distance allowed from wall is ");
      Serial.println(DistanceToWall);
      Serial.print("Left IR values Long Sensor: ");
      Serial.println(irLeftLVal);
      Serial.print("Left IR values Short Sensor: ");
      Serial.println(irLeftSVal);

      if (irLeftLVal >= irLeftSVal) {
        moveRobot(0, 0, 0.3);  //Back Left is too far away, rotate right
      }
      else if (irLeftLVal <= irLeftSVal) {
        moveRobot(0, 0, -0.3); //Front Left is too far away, rotate left
      }
      Serial.print("Have a look: ");
      Serial.println(abs(irLeftLVal - irLeftSVal));

      //Extents exit condition and guarentees that robot is alligned and correctly positioned.
      if (abs(irLeftLVal - irLeftSVal) < 0.5) {
        Serial.print("Counter is now: ");
        Serial.println(counter);
        counter = counter + 1;
      } else {
        Serial.println("Resetting counter");
        counter = 0;
      }

      delay(50);
    } while (counter < 5);
    Serial.println("Exiting Align function");
  }
  moveRobot(0, 0, 0);
  delay(1000);
}

//======================================================================================
bool checkShortLong() {
  bool NeedToTurn = false;

  rotateRobot(90, cw); //Turn "90 degrees" clockwise given 1 rad/s
  //alignWall(leftWall);
  float a = GetSonarDist();
  Serial.print("Value A:");
  Serial.println(a);

  rotateRobot(90, cw); //Turn "90 degrees" clockwise given 1 rad/s
  //alignWall(rightWall);
  float b = GetSonarDist();
  Serial.print("Value B:");
  Serial.println(b);

  //Set NeedToTurn to 1;
  if (a > b) {
    NeedToTurn = true;
    Serial.println("Robot needs to turn");
  } else {
    Serial.println("Robot does not need to turn");
  }
  return NeedToTurn;
}

//======================================================================================
void forwardMovement(float distanceToWall) { //the forward/backwards motion after it has done a strafe movement
  do {
    Serial.println("GOING STRAIGHT YOU FUCK");
    moveRobot(0, 0.44, 0);
    uSensor = GetSonarDist();
    /*
      if ((getIRDistance(irLeftS) < 17) || (getIRDistance(irLeftL) < 17)) {
      moveRobot(0.2, 0.2, 0);//Move robot forwards + right
      delay(50);
      }
      if ((getIRDistance(irLeftS) > 17) || (getIRDistance(irLeftL) > 17)) {
      moveRobot(-0.2, 0.2, 0);//Move robot forwards + left
      delay(50);
      }
      if ((getIRDistance(irRightS) < 17) || (getIRDistance(irRightL) < 17)) {
      moveRobot(-0.2, 0.2, 0);//Move robot forwards + left
      delay(50);
      }
      if ((getIRDistance(irRightS) > 17) || (getIRDistance(irRightL) > 17)) {
      moveRobot(0.2, 0.2, 0);//Move robot forwards + right
      delay(50);
      }
    */
  } while (uSensor < distanceToWall);

  moveRobot(0, 0, 0);
  delay(1000);
}
//======================================================================================
//void rightUturn (float DistanceToWall) {
//  Serial.println("Right turn baby");
//  // Right U-turn
//  rotateRobot(90, cw);    //Turn right
//  alignWall(leftWall);
//  forwardMovement(DistanceToWall);// Moves forwards a little bit
//  rotateRobot(90, cw);    //Turn Right    !!!!!!!!!!!!!!!!!!!!!DOES NOT HAVE AN ALIGN FUNCTION:NO WALL
//}
//======================================================================================
//void leftUturn (float DistanceToWall) {
//  Serial.println("Left turn baby");
//  // Left U-turn
//  rotateRobot(90, ccw);   //Turn Left
//  alignWall(rightWall);
//  forwardMovement(DistanceToWall);// Moves forwards a little bit
//  rotateRobot(90, ccw);   //Turn Left    !!!!!!!!!!!!!!!!!!!!!DOES NOT HAVE AN ALIGN FUNCTION:NO WALL
//}
