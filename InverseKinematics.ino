
float radius = 0.0265;
float lowL = 0.09;
float upL = 0.08;
float L = lowL + upL;

//Function controls the movement of the motors, making it move forward, backwards, left and right
//======================================================================================
void moveRobot(float vx, float vy, float wz) {

  //vx commands0
  //vy commands1
  //wz commands2

  float wheelSpeed1 = constrain((vx + vy + (wz * (L))) / radius * 500 / 16.755, -500, 500); //left front
  float wheelSpeed2 = constrain((vx - vy + (wz * (L))) / radius * 500 / 16.755, -500, 500); //right front
  float wheelSpeed3 = constrain((-1 * vx + vy + (wz * (L))) / radius * 500 / 16.755, -500, 500); //left rear
  float wheelSpeed4 = constrain((-1 * vx - vy + (wz * (L))) / radius * 500 / 16.755, -500, 500); //right rear

  //  Serial.print(wheelSpeed1);
  //  Serial.print("");
  //  Serial.print(wheelSpeed2);
  //  Serial.print("");
  //  Serial.print(wheelSpeed3);
  //  Serial.print("");
  //  Serial.print(wheelSpeed4);
  //  Serial.println();
  //  Serial.println("==================================");
  //
  left_font_motor.writeMicroseconds(1500 + wheelSpeed1);//left front
  right_font_motor.writeMicroseconds(1500 + wheelSpeed2);//right front
  left_rear_motor.writeMicroseconds(1500 + wheelSpeed3);//left rear
  right_rear_motor.writeMicroseconds(1500 + wheelSpeed4);//right rear
}

//======================================================================================
