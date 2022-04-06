//File Contains all the functions that are used for the basic motor movement testing

//----------------------Motor moments------------------------
////The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control
//
//void forward()
//{
//  left_font_motor.writeMicroseconds(1500 + speed_val);
//  left_rear_motor.writeMicroseconds(1500 + speed_val);
//  right_rear_motor.writeMicroseconds(1500 - speed_val);
//  right_font_motor.writeMicroseconds(1500 - speed_val);
//}
//
////======================================================================================
//
//void reverse ()
//{
//  left_font_motor.writeMicroseconds(1500 - speed_val);
//  left_rear_motor.writeMicroseconds(1500 - speed_val);
//  right_rear_motor.writeMicroseconds(1500 + speed_val);
//  right_font_motor.writeMicroseconds(1500 + speed_val);
//}
//
////======================================================================================
//
//void ccw ()
//{
//  left_font_motor.writeMicroseconds(1500 - speed_val);
//  left_rear_motor.writeMicroseconds(1500 - speed_val);
//  right_rear_motor.writeMicroseconds(1500 - speed_val);
//  right_font_motor.writeMicroseconds(1500 - speed_val);
//}
//
////======================================================================================
//
//void cw ()
//{
//  left_font_motor.writeMicroseconds(1500 + speed_val);
//  left_rear_motor.writeMicroseconds(1500 + speed_val);
//  right_rear_motor.writeMicroseconds(1500 + speed_val);
//  right_font_motor.writeMicroseconds(1500 + speed_val);
//}
//
////======================================================================================
//
//void strafe_left ()
//{
//  left_font_motor.writeMicroseconds(1500 - speed_val);
//  left_rear_motor.writeMicroseconds(1500 + speed_val);
//  right_rear_motor.writeMicroseconds(1500 + speed_val);
//  right_font_motor.writeMicroseconds(1500 - speed_val);
//}
//
////======================================================================================
//
//void strafe_right ()
//{
//  left_font_motor.writeMicroseconds(1500 + speed_val);
//  left_rear_motor.writeMicroseconds(1500 - speed_val);
//  right_rear_motor.writeMicroseconds(1500 - speed_val);
//  right_font_motor.writeMicroseconds(1500 + speed_val);
//}
//
////======================================================================================
//
////======================================================================================
//void stop() //Stop
//{
//  left_font_motor.writeMicroseconds(1500);
//  left_rear_motor.writeMicroseconds(1500);
//  right_rear_motor.writeMicroseconds(1500);
//  right_font_motor.writeMicroseconds(1500);
//}
//
////======================================================================================
//
//void disable_motors()
//{
//  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
//  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
//  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
//  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off
//
//  pinMode(left_front, INPUT);
//  pinMode(left_rear, INPUT);
//  pinMode(right_rear, INPUT);
//  pinMode(right_front, INPUT);
//}
//
////======================================================================================
