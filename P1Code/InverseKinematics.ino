
float radius = 0.0265;
float lowL = 0.09;
float upL = 0.08;
float L = lowL + upL;

//Carl and Suvarna's implementaiton of moveRobot (using the velocities[] array) for EKF convenience
//======================================================================================
void moveRobot(velocities v) {
  float wheelSpeed1 = constrain((v.x + v.y + -1*(v.z * (L))) / radius * 500 / 16.755, -500, 500); //left front
  float wheelSpeed2 = constrain((v.x - v.y + -1*(v.z * (L))) / radius * 500 / 16.755, -500, 500); //right front
  float wheelSpeed3 = constrain((-1 * v.x + v.y + -1*(v.z * (L))) / radius * 500 / 16.755, -500, 500); //left rear
  float wheelSpeed4 = constrain((-1 * v.x - v.y + -1*(v.z * (L))) / radius * 500 / 16.755, -500, 500); //right rear

  left_font_motor.writeMicroseconds(1500 + wheelSpeed1);//left front
  right_font_motor.writeMicroseconds(1500 + wheelSpeed2);//right front
  left_rear_motor.writeMicroseconds(1500 + wheelSpeed3);//left rear
  right_rear_motor.writeMicroseconds(1500 + wheelSpeed4);//right rear
}
