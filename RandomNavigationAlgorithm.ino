

//-------------------------Start includes--------------------------
#include <Wire.h>
#include <Zumo32U4.h>
#include <TurnSensor.h>

Zumo32U4IMU imu;
Zumo32U4LCD lcd;
Zumo32U4Motors Motors;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer Buzzer;
Zumo32U4LineSensors lineSensors;

Zumo32U4ButtonA ButtonA;
Zumo32U4ButtonB ButtonB;
Zumo32U4ButtonC ButtonC;

//======================== End includes =======================

// ------------------- Start Robot components-------------------

// Ting vi skal bruge til line-sensor:

// Vi har en bool, som vi sætter til false, så den køre indtil den er true.
bool lineFound = false;

// ============================ End Robot components============================

// ------------------- start global variables-------------------

// this is how fast the zumo will turn.
int turnSpeed = 150;
// this is how fast the zumo will drive.
int driveSpeed = 100;

// this is an array where the values read from the linesensors will be put.
uint16_t lineSensorValues[5];

int sensorMin[5] = {1023, 1023, 1023, 1023, 1023}; // minimum sensor value
int sensorMax[5] = {0, 0, 0, 0, 0};                // maximum sensor value

// an array where the first colum is for turning right or left, and the second is for wich random interval it should turn.
int TurnAndInterval[2];

// for the function that detect the barrierer
int black;
int gray;
int oldgray;
int BLACK;
int GRAY;

int randNumber;

enum State
{
  leftFront,
  rightFront
};
State waveState = leftFront;

int currentSpeedLeft = driveSpeed;
int currentSpeedRight = driveSpeed;
int error;
int correction;

int countsLeft = encoders.getCountsAndResetLeft();
int countsRight = encoders.getCountsAndResetRight();

#define Kp 1
#define STRAIGHTFACTOR 0.985

// ============================ End global variables =======================

//------------------------Start Support functions-------------------

// _______ support functions _____
/**
 * @brief The funtions calculates the angle from last rest of the gyro.
 *
 * @return the return from this funciton is the amount of degrees the zumo have turned, from when the gyro was last rest.
 */
int32_t getAngle()
{
  turnSensorUpdate();
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}

/**
 * @brief this function makes a random number
 * between 91-180 and 181-270
 *
 * @param i is for which intervarl of randomnumbers the function gives back: if it is 1 than the interval is 91-180. if it is 2 than the interval 181-270.
 */
int numberGen()
{

  randNumber = random(91, 180); // Pick a random number between 91 and 179

  return randNumber;
}

/**
 * @brief turns the zumo left by a certain amount of degrees.
 *
 * @param degrees is the amount the zumo turns.
 */
void turnLeft(int degrees)
{
  turnSensorReset();
  Motors.setSpeeds(-turnSpeed, turnSpeed);
  int angle = 0;
  do
  {
    delay(1);
    turnSensorUpdate();
    angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
    // lcd.gotoXY(0, 0);
    // lcd.print(angle);
    // lcd.print(" ");
  } while (angle < degrees);
  Motors.setSpeeds(0, 0);
}

/**
 * @brief turns the zumo right by a certain amount of degrees.
 *
 * @param degrees is the amount the zumo turns.
 */
void turnRight(int degrees)
{
  turnSensorReset();
  Motors.setSpeeds(turnSpeed, -turnSpeed);
  int angle = 0;
  do
  {
    delay(1);
    turnSensorUpdate();
    angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
    // lcd.gotoXY(0, 0);
    // lcd.print(angle);
    // lcd.print(" ");
  } while (angle > -degrees);
  Motors.setSpeeds(0, 0);
}

// ________ for setup loop ________

/**
 * @brief this function finds the lowest line value and maximum linevalue and returns them in a array.
 *
 */
void setupCalibrateLineSensors()
{
  lineSensors.read(lineSensorValues);
  for (int i = 0; i < 5; i++)
  {

    // record the maximum sensor value
    if (lineSensorValues[i] > sensorMax[i])
    {
      sensorMax[i] = lineSensorValues[i];
    }

    // record the minimum sensor value
    if (lineSensorValues[i] < sensorMin[i])
    {
      sensorMin[i] = lineSensorValues[i];
    }
  }
}

// ________ for main loop_______

void driveStrightDammit()
{

  Motors.setSpeeds(currentSpeedLeft, currentSpeedRight);
  // do {
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();
  error = countsLeft - STRAIGHTFACTOR * countsRight;
  correction = Kp * error;
  currentSpeedRight = driveSpeed + correction;
  Motors.setSpeeds(currentSpeedLeft, currentSpeedRight);
  //}  while(countsLeft<DISTANCE&&countsRight<DISTANCE);

  // Motors.setSpeeds(0,0);
}

void calibratedLineSensors()
{
  lineSensors.read(lineSensorValues);

  for (int i = 0; i < 5; i++)
  {

    // in case the sensor value is outside the range seen during calibration
    lineSensorValues[i] = constrain(lineSensorValues[i], sensorMin[i], sensorMax[i]);

    // apply the calibration to the sensor reading
    lineSensorValues[i] = map(lineSensorValues[i], sensorMin[i], sensorMax[i], 0, 3000);
  }
}

/**
 * @brief This function decides if to turn right or left with the input of an array,
 * and how much it needs to turn
 *
 * @param a : if a is 1 then it turns left in a interval, if 2 turns right in interval, if 3 turns left 90, if 4 turns right 90.
 * @param b : 1 will give the interval 91-180 and 2 will give the interval 181-270.
 */
void Gyro(int a)
{
  // resest the zumos gyro
  turnSensorReset();

  switch (a)
  {
  case 1:
    // it turns left in the interval 91-180.
    turnLeft(numberGen());
    break;
  case 2:
    // it turns right in the interval 91-180.
    turnRight(numberGen());
    break;
  case 3:
    turnLeft(90);
    break;
  case 4:
    turnRight(90);
    break;

  default:
    Serial.print("ERROR: Gyro function");
    break;
  }
}

/**
 * @brief this function makes counts how fare the zumo drives and how fast it drives.
 *
 * @param a is how many counts it should check
 * @param b how fast it should drive. max speed -400 - 400.
 */
void DriveDistance(int a, int b)
{

  encoders.getCountsAndResetLeft();

  Motors.setSpeeds(b, b);
  delay(200);
  bool britney = true;

  while (abs(encoders.getCountsLeft()) < abs(a) && britney == true)
  {
    lcd.clear();
    lcd.print(encoders.getCountsLeft());
    lcd.gotoXY(1, 1);
    lcd.print(BarrierAndCornerDetect());
    Motors.setSpeeds(b, b);
    delay(20);

    /*
        if (encoders.getCountsLeft() > a)
        {
          Buzzer.playFrequency(400, 100, 10);
        }
    */
    if (!(BarrierAndCornerDetect() == 'w'))
    {
      Buzzer.playFrequency(1200, 100, 12);
      britney = false;
    }
  }
  Motors.setSpeeds(0, 0);
}

char BarrierAndCornerDetect()
{
  calibratedLineSensors();

  black = 0;
  gray = 0;

  delay(20);
  for (int i = 0; i < 5; i++)
  {

    if (lineSensorValues[i] > 2000)
    {
      black = 1;
    }
    else if (400 < lineSensorValues[i] && lineSensorValues[i] < 700)
    {
      gray = 1;
    }
  }

  if (black == 1)
  {
    oldgray = 0;
    return 'b';
  }
  else if (gray == 1 && !black == 1)
  {
    if (oldgray == 1)
    {
      return 'g';
    }

    oldgray = 1;
  }
  else
  {
    oldgray = 0;
    return 'w';
  }
}

// ============================ End Support functions ============================

//------------------ Start of The code---------------------------

void setup()
{

  encoders.init();

  // begins terminal
  Serial.begin(9600);

  // turns on linesensors
  lineSensors.emittersOn();
  lineSensors.initFiveSensors();

  // waits for the button A to be pressed on the zumo
  ButtonA.waitForButton();
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  /*calibrate during the first 2.5 seconds*/
  for (int i = 0; i < 500; i++)
  {
    // lineSensors.calibrate();
    setupCalibrateLineSensors();
    if (i == 251)
    {
      Buzzer.playFrequency(440, 100, 10);
    }

    delay(10);
  }

  // signal the end of the calibration period
  digitalWrite(13, LOW);

  // waits for the button A to be pressed on the zumo
  ButtonB.waitForButton();

  digitalWrite(13, HIGH);
  turnSensorSetup();
  delay(500);
  turnSensorReset();

  digitalWrite(13, LOW);

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  int seed = millis();
  randomSeed(seed);

/*
  while (true)
  {
    lcd.clear();
    lcd.print(numberGen());
    delay(1000);
  }

  lcd.clear();
  lcd.print(numberGen());
  delay(1000);

  lcd.clear();
  lcd.print(numberGen());
  delay(1000);*/
}

void loop()
{
  lcd.clear();
  if (waveState == leftFront)
  {

    lcd.print("  <---  ");
  }
  else if (waveState == rightFront)
  {

    lcd.print("  --->  ");
  }

  lcd.gotoXY(0, 1);
  lcd.print(BarrierAndCornerDetect());

  lcd.gotoXY(2, 1);
  lcd.print(getAngle());

  calibratedLineSensors();

  Serial.println((String) "line " + (String)lineSensorValues[0] + ' ' + (String)lineSensorValues[1] + ' ' + (String)lineSensorValues[2] + ' ' + (String)lineSensorValues[3] + ' ' + (String)lineSensorValues[4]);

  driveStrightDammit();

  switch (BarrierAndCornerDetect())
  {
  case 'g':
    Buzzer.playFrequency(200, 100, 10);
    DriveDistance(-350, -100);

    if (waveState == rightFront)
    {
      DriveDistance(-125, -100);
      // turn left in the interval 91-180.
      Gyro(1);
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      turnSensorReset();

      waveState = leftFront;
    }
    else if (waveState == leftFront)
    {
      DriveDistance(-125, -100);
      // turn right in the interval 91-180.
      Gyro(2);
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      turnSensorReset();
    }
    break;

  case 'b':
    Buzzer.playFrequency(800, 100, 10);
    DriveDistance(-350, -100);

    if (waveState == rightFront)
    {
      // turn right 90.
      Gyro(4);

      DriveDistance(800, 100);
      if (!(BarrierAndCornerDetect() == 'w'))
      {
        DriveDistance(-125, -100);
      }

      Gyro(4);
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();

      waveState = leftFront;

      turnSensorReset();
    }
    else if (waveState == leftFront)
    {

      // turn left 90.
      Gyro(3);

      DriveDistance(800, 100);
      if (!(BarrierAndCornerDetect() == 'w'))
      {
        DriveDistance(-125, -100);
      }
      Gyro(3);
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();

      waveState = rightFront;
      turnSensorReset();
    }

    break;

  default:
    break;
  }
}

// ============================ End of The code ============================

