//File Contains all the functions related to the motor movement commands
//======================================================================================
void moveToCorner(float distanceToWall) {
  Serial.println("Entered move to corner function");
  do {
    moveRobot(setV(0, 0.44, 0));

    if ((getIRDistance(irLeftS) < 12) || (getIRDistance(irLeftL) < 12)) {
      moveRobot(setV(0.2, 0.2, 0));//Move robot forwards + left
      delay(50);
    }
    if ((getIRDistance(irLeftS) > 12) || (getIRDistance(irLeftL) > 12)) {
      moveRobot(setV(-0.2, 0.2, 0));//Move robot forwards + right
      delay(50);
    }
  } while (GetSonarDist() > distanceToWall);//Stop when ultrasonic sensor has detected that distance wall has been reached.
  moveRobot(setV(0, 0, 0));
  delay(intervalDelay);
}

//======================================================================================
void rotateRobot(float angle, bool angleDirection) {//update function to use both gyro and ir sensors
  float presentAngle = 0; //Reset current angle
  float error=0;
  float output=0;
  
  resetGyro();
  
  
  if (angleDirection == 0) {         //Rotating CCW
    Serial.println("Rotate Counter-Clockwise");
    do {
      moveRobot(setV(0, 0, 1));
      //get current angle and print
      presentAngle = getCurrentAngle();
      Serial.print("This is the currentAngle: ");
      Serial.println(presentAngle);
      Serial.print("This is the Angle we want to reach: ");
      Serial.println(angle);
    } while ( (presentAngle < (angle)) || (presentAngle >= 350) );
  }

  else if (angleDirection == 1) {   //Rotating CW
    Serial.println("Rotate Clockwise");
    do {
      moveRobot(setV(0, 0, -1));
      //get current angle and print
      presentAngle = getCurrentAngle();
//      Serial.print("This is the currentAngle: ");
//      Serial.println(presentAngle);
//      Serial.print("This is the Angle we want to reach: ");
//      Serial.println(((360 - angle) * 1.07));
    } while (((presentAngle) > ((360 - angle) * 1.08)) ||(presentAngle<=10));
    Serial.print("This is the currentAngle: ");
    Serial.println(presentAngle);
  }
  moveRobot(setV(0, 0, 0));
  delay(intervalDelay);
}

//======================================================================================

void alignWall(bool senseDirection, float DistanceToWall) {
  if (senseDirection) {
    int counter = 0;
    do {
      irRightLVal = getIRDistance(irRightL);
      irRightSVal = getIRDistance(irRightS);
      if (irRightLVal >= irRightSVal) {
        moveRobot(setV(0, 0, 0.5)); //Back right is too far away, rotate left
      }
      else if (irRightLVal <= irRightSVal) {
        moveRobot(setV(0, 0, -0.5)); //Front right is too far away,rotate right
      }
      //Extents exit condition and guarentees that robot is alligned and correctly positioned.
      if (abs(irRightLVal - irRightSVal) < 0.5) {
        Serial.print("Counter is now: ");
        Serial.println(counter);
        counter = counter + 1;
      } else {
        Serial.println("Resetting counter");
        counter = 0;
      }
      delay(50);
    } while (counter < 5);
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
        moveRobot(setV(0,0,-0.3));  //Back Left is too far away, rotate right
      }
      else if (irLeftLVal <= irLeftSVal) {
        moveRobot(setV(0, 0, 0.3)); //Front Left is too far away, rotate left
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
  moveRobot(setV(0,0,0));
  delay(intervalDelay);
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
 void yMovement(float distanceToWall, float KP) { 
  float currentDistance = 0;
  float error=0;
  float output=0;
  
  currentDistance=GetSonarDist();
  error = distanceToWall - currentDistance;
  output =abs( KP * error);
  
  do {
    
    if (error<0){ //MOVE FOWARD
    moveRobot(setV(0, constrain(0.44*output,0,0.44), 0));
    currentDistance= GetSonarDist();
    }
    else if(error>0){ //MOVE BACKWARD
      moveRobot(setV(0, constrain(-0.44*output,-0.44,0), 0));
      currentDistance= GetSonarDist();
    }
    
    //P Controller
    error = distanceToWall - currentDistance;
    output =abs( KP * error);
    
  } while( (currentDistance!= distanceToWall) && (abs(error) > 5)); 
  
  Serial.print("Distance to Reach");
  Serial.println(distanceToWall);
  Serial.print("Current Distance");
  Serial.println(currentDistance);
  moveRobot(setV(0, 0, 0));
  delay(intervalDelay);
}
//======================================================================================
void xMovement(float distanceToWall, bool sideOfRobot, float KP, float sonicDistance, float KP_SONIC, float KP_GYRO) { 
  float error=0;
  float output=0;
  float yDist=0;
  float yDiff=0;
  float yOutput=0;
  float xDiff=0;
  float zOutput=0;
  
  

  if (sideOfRobot==0){ //LEFT SIDE OF ROBOT

    //USE ONLY SHORT IR SENSOR
    irLeftSVal = getIRDistance(irLeftS);
    error = irLeftSVal -distanceToWall;

    yDist=GetSonarDist();
    yDiff = yDist-sonicDistance;

    irLeftLVal = getIRDistance(irLeftL);
    xDiff = irLeftSVal-irLeftLVal;

    do {
      //P CONTROLLER
      output = KP*error;
      yOutput = KP_SONIC*yDiff;
      zOutput = KP_GYRO *xDiff;

      

      //STILL HAS SLIPPAGE
      moveRobot (setV(constrain(-0.44*output,-0.44,0.22),constrain(0.44*yOutput,-0.44,0.44),constrain(1*zOutput,-1,1)));
      irLeftSVal = getIRDistance(irLeftS);
      error = irLeftSVal -distanceToWall;

      yDist=GetSonarDist();
      yDiff = yDist-sonicDistance;

      irLeftLVal = getIRDistance(irLeftL);
      xDiff = irLeftSVal-irLeftLVal;
      
      //Checking LONG IR DOES NOT GO OUT OF RANGE
      if ((irLeftLVal<10)||(irLeftLVal>30)){
      xDiff =0;
      }
      Serial.print("Left Distance");
      Serial.println(irLeftSVal);
      
    }while((irLeftSVal != distanceToWall) && (abs(error)>3));
    
    
  }else if (sideOfRobot ==1){ //RIGHT SIDE OF ROBOT

    //USE ONLY SHORT IR SENSOR
    irRightSVal = getIRDistance(irRightS);
    error = irRightSVal-distanceToWall;

    yDist=GetSonarDist();
    yDiff = yDist-sonicDistance;
    do{
      //P CONTROLLER
      output = KP*error;
      yOutput = KP_SONIC*yDiff;
      
      moveRobot (setV(constrain(0.44*output,-0.44,0.44),constrain(0.44*yOutput,-0.44,0.44),0));
      irRightSVal = getIRDistance(irRightS);
      error = irRightSVal-distanceToWall;

      yDist=GetSonarDist();
      yDiff = yDist-sonicDistance;
      Serial.print("Right Distance");
      Serial.println(irRightSVal);
    }while ((irRightSVal != distanceToWall) && (abs(error)>3));
    
  }
  moveRobot(setV(0, 0, 0));
  delay(intervalDelay);
}
//======================================================================================
void rightUturn (float DistanceToWall) {
  Serial.println("Right turn baby");
  // Right U-turn
  rotateRobot(90, cw);    //Turn right
  alignWall(leftWall,10);
  yMovement(DistanceToWall,KP_ULTRASONIC);// Moves forwards a little bit
  rotateRobot(90, cw);    //Turn Right    !!!!!!!!!!!!!!!!!!!!!DOES NOT HAVE AN ALIGN FUNCTION:NO WALL
}
//======================================================================================
void leftUturn (float DistanceToWall) {
  Serial.println("Left turn baby");
  // Left U-turn
  rotateRobot(90, ccw);   //Turn Left
  alignWall(rightWall,10);
  yMovement(DistanceToWall, KP_ULTRASONIC);// Moves forwards a little bit
  rotateRobot(90, ccw);   //Turn Left    !!!!!!!!!!!!!!!!!!!!!DOES NOT HAVE AN ALIGN FUNCTION:NO WALL
}
