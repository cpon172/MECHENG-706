//Main code that contains all the intial variables aswell as being the location where other file functions are called.

#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "BasicLinearAlgebra.h"
#include "RunningMedian.h"
using namespace BLA;

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//======================================================================================
//SHIELD PINOUTS

#define BLUETOOTH_RX 10
#define BLUETOOTH_TX 11 // Serial Data output pin

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

//IR Sensor Pins
int irLeftL = 13;
int irRightL = 14;
int irLeftS = 12;
int irRightS = 15;

//IR SENSOR VALUES
float irLeftLVal = 0;
float irLeftSVal = 0;
float irRightLVal = 0;
float irRightSVal = 0;

//Gyro pin
int gyroPin = A3;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

//======================================================================================
//BLUETOOTH SETUP
#define STARTUP_DELAY 10 // Seconds
#define LOOP_DELAY 10 // miliseconds
#define SAMPLE_DELAY 10 // miliseconds

// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0

// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

//======================================================================================
//VARIABLES

//Structure for velocities
struct velocities {
  float x;
  float y;
  float z;
};

velocities v = {0, 0, 0}; //initialise a velocities structure called v

//State machine states
enum STATE { INITIALISING, RUNNING, STOPPED };

int speed_val = 300; //-500 to 500
int speed_change;

//Serial Pointer
HardwareSerial *SerialCom;

int pos = 0; //Variable for turret servo motor

int wallNumber; //for EKF and data association

int isrCount = 0; //counter for ISR
int isRunning = 1; //Variable used for running the program
int i = 0; //index for how many paths have been done
int SLAM = 0; //flag for enabling slam

//Turning definitions
#define cw 1
#define ccw 0
#define rightTurn 1
#define leftTurn 0
#define rightWall 1
#define leftWall 0

float intervalDelay = 500; //Pause time between each movement command

//======================================================================================
//ULTRASONIC VARIABLES

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

//Takes 7 sample readings then finds the median.
RunningMedian samples = RunningMedian(10);

//======================================================================================
//GYROSCOPE VARIABLES

int sensorValue = 0; // read out value of sensor
float gyroSupplyVoltage = 5; // supply voltage for gyro

float gyroZeroVoltage = 0; // the value of voltage when gyro is zero
float gyroSensitivity = 0.007; // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5; // because of gyro drifting, defining rotation angular velocity less than this value will be ignored
float gyroRate = 0; // read out value of sensor in voltage
float currentAngle = 0; // current angle calculated by angular velocity integral on
byte serialRead = 0; // for serial print control
float gyroAngle = 0;

int T = 50;// gyroscope delay time (For Jason and Aniqah)

//======================================================================================
//EKF MATRICES AND CONTROL COMMANDS

// Sensor KF
//(in order): sonar, irLeftL, irLeftS, irRightL, irRightS
float Mut_prevlist[5] = {0, 0, 0, 0, 0}; //Initial/mean value of the 5 sensors
float Sigmat_prevlist[5] = {999, 999, 999, 999, 999}; //Initial Covariance of the 5 sensors
float RList[5] = {1, 1, 1, 1, 1};
float QList[5] = {100, 100, 100, 100, 100}; // Change the value of sensor noise to get different KF performance

//velocities x,y,z in that order. These velocities are used as inputs for motor commands but more importantly, as inputs for the extended kalman filter
//(For Carl and Suvarna)
//float velocities[3] = {0, 0, 0};


//EFK MATRICES
Matrix<11> currentState; //x, y, theta, land1x, land1y, land2x, land2y, land3x, land3y, land4x, land4y
Matrix<11> prevState;

Matrix<11, 11>covarianceMatrix;
RefMatrix<3, 3, Array<11, 11>> robotCov(covarianceMatrix.Submatrix<3, 3>(0, 0)); //Robot Cov matrix
RefMatrix<8, 8, Array<11, 11>> landmarkCov(covarianceMatrix.Submatrix<8, 8>(3, 3)); //Landmark Cov matrix
RefMatrix<8, 3, Array<11, 11>> blCov(covarianceMatrix.Submatrix<8, 3>(3, 0)); //Bottom Left Cov matrix
RefMatrix<3, 8, Array<11, 11>> trCov(covarianceMatrix.Submatrix<3, 8>(0, 3)); //Top Right Cov matrix

//RefMatrix<4, 4, Array<11, 11>> apples(covarianceMatrix.Submatrix<4, 4>(4, 2));

//======================================================================================
#define KP_ULTRASONIC 0.01

//======================================================================================
void setup(void)
{
  resetGyro();
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  //Setting up IR Sensors as inputs
  pinMode(irLeftL, INPUT);
  pinMode(irLeftS, INPUT);
  pinMode(irRightL, INPUT);
  pinMode(irRightS, INPUT);

  //Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  BluetoothSerial.begin(115200);

  //EKF MATRICES SETUP (INITIALIZATION)=================================================
  currentState.Fill(0);
  prevState.Fill(0);
  covarianceMatrix.Fill(0);

  for (int i = 0; i < 8; i++) { //Diagonal matrix for the landmark covariance matrix
    landmarkCov(i, i) = 999;
  }

  Serial << "Current State: " << currentState << '\n';
  Serial << "Initial Robot Covariance Matrix: " << robotCov << '\n';
  Serial << " Iniital Landmark covariance: " << landmarkCov << '\n';

  //====================================================================================
  //TIMER INTERRUPT SETUP FOR USE OF THE EKF
  cli();
  OCR2A = 252; //CTC OCR1A top value, upon overflow, 1 second has passed
  //
  TCCR2B |= (1 << WGM21) | (1 << CS22) | (1 << CS21) | (1 << CS20); //config CTC mode, prescaler 1024
  TIMSK2 |= (1 << OCIE2A);
  sei();
}

//======================================================================================

void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
  };
}

//======================================================================================

STATE running() {

  static unsigned long previous_millis;

  //read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();

#ifndef NO_READ_GYRO
    GYRO_reading();
#endif

    //#ifndef NO_HC-SR04
    //    HC_SR04_range();
    //#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif
    turret_motor.write(pos);
    if (pos == 0)
    {
      pos = 45;
    }
    else
    {
      pos = 0;
    }
  }

  //ENTER CODE HERE!======================================================================================================================

  //CHECKING CODE HERE+++++++++++++++++++++++++++++++++
  //float Distance =GetSonarDist();
  //yMovement(50,KP_ULTRASONIC);
  //rotateRobot(90,ccw);
  //strafeRight();
  //xMovement(8, leftWall, 0.01, 30, 0.01, 0.01);  //SEMI GOOD UNTIL THE END STARTS TO TURN

  //*****************************************************************************
  /* Carl and Suvarna's requirements for the EKF
  The EKF requires the current velocities that are being imposed upon the robot to update its coordinates on the table.
  This is done by modifying the velocities struct type 'v' (line 75), using the setV function. The setV function is ONLY
  used inside the moveRobot() function as a paramter. 
  
  e.g. to make the robot move forward 0.2ms^-1 with moveRobot:
    moveRobot(setV(0, 0.2, 0));
  */
  
 
  //*****************************************************************************

  //Jason and Aniqah's path following stuff
    float maximumDistanceToWall = 4;
  
  
    //intial movement code
    Serial.println("Intial movement to wall");
    yMovement(maximumDistanceToWall, KP_ULTRASONIC); //Turn
    Serial.println("Rotate 90 degrees");
    //Turn "90 degrees" clockwise given 1 rad/s
    rotateRobot(90,cw);
    Serial.println("Move towards corner");
    alignWall(leftWall,10);
    moveToCorner(maximumDistanceToWall);


    int initialMovement = 2;
    //Main zig zag motion
    Serial.println("checks the Short and Long distance");
    //This might be iffy make sure variables being returned are correct
    if (checkShortLong()==true) { //If robot needs to turn initially (ie on left side of table)
    Serial.println("Right side of table");
    rotateRobot(90, ccw);
    alignWall(leftWall,10);
    yMovement(maximumDistanceToWall, KP_ULTRASONIC);
    initialMovement = rightTurn;
    } else { //If robot does not need to turn intially (ie on rigjht side of table)
    Serial.println("Left side of table");
    alignWall(rightWall, 10);
    yMovement(maximumDistanceToWall, KP_ULTRASONIC);
    initialMovement = leftTurn;
//    SLAM = 1;
    }
    Serial.println("Entered Zig Zag movemnt");
    for (float DistanceToWall = 100; DistanceToWall >= 10; DistanceToWall - 10) {
    while (DistanceToWall > 10) {
      if (initialMovement == rightTurn) {
        rightUturn(DistanceToWall);
        yMovement(maximumDistanceToWall, KP_ULTRASONIC);
        initialMovement = leftTurn;//Always alternates between left and right Turns
      } else {
        leftUturn(DistanceToWall);
        yMovement(maximumDistanceToWall, KP_ULTRASONIC);
        initialMovement = rightTurn;
      }
    }
    }
  moveRobot(setV(0, 0, 0));
  delay(1000000000);
  //return STOPPED;
}

//======================================================================================

STATE initialising() {
  //initialising
  //  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  //  SerialCom->println("Enabling Motors...");
  enable_motors();
  //  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

//======================================================================================

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

//======================================================================================

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}
//======================================================================================
