
//----------- start includes ---------
#include <Zumo32U4.h>

Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

//=========== end includes ===========

//---------- start main code constants ------------

int motorSpeed = 100;
bool lineDetected = false;
int threshHold = 300;
bool turnLine = false;
bool iAmHome = false;



// ========== end main code constants =============

//---------- start calibration constants -----------

uint16_t lineSensorValues[5];

//=========== end calibration constants ===========

//--------- start calibration variables ------------
int sensorMin[5] = {1023, 1023, 1023, 1023, 1023}; // minimum sensor value
int sensorMax[5] = {0, 0, 0, 0, 0};                // maximum sensor value

int black;
int gray;
int oldgray;

//======== end global vairables ===========


//------------ start calibration function ----------

uint16_t *SetupCalibrateLinesensors()
{

  int i;
  for (i = 0; i < 5; i++)
  {
    lineSensors.read(lineSensorValues);

    //records the minimum sensor value
    if (lineSensorValues[i] > sensorMax[i])
    {
      sensorMax[i] = lineSensorValues[i];
    }

    // records the minimum sensor value
    if (lineSensorValues[i] < sensorMin[i])
    {
      sensorMin[i] = lineSensorValues[i];
    }
  }

  // return lineSensorValues;
}

//============ end calibration function ========


//------------ start line detection function ------
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
//========== end line detection function ======

//---------- start program -----------------

//------- setup loop
void setup()
{
  // turn on LED to signal the start of the calibration period:

  Serial.begin(9600);

  lineSensors.emittersOn();
  lineSensors.initFiveSensors();

  buttonA.waitForButton();
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  /*calibrate during the first 2.5 seconds*/
  for (int i = 0; i < 600; i++) //Calibrates the sensor 600 times, first 300 for black and then 300 for white
  {

    SetupCalibrateLinesensors();

    if (i == 301) //When 300 measurements has been done
    {
      buzzer.playFrequency(440, 100, 10); //Beeps to tell to move the robot to white
      motors.setSpeeds(100, 100);
      delay(300);
      motors.setSpeeds(0, 0);
    }
  }

  // LED turns of signal at the end of the calibration period
  digitalWrite(13, LOW);
}

//------ main loop
void loop()
{
  calibratedLineSensors();
  buttonA.waitForButton();
  while (lineDetected == false) {
    motors.setSpeeds(motorSpeed, motorSpeed); //The Zumo starts driving forward
    calibratedLineSensors(); //Checks if the line has been detected
    if (lineSensorValues[2] > threshHold) { //When the line has been detected exit the loop
      lineDetected = true;
    }
  }
  motors.setSpeeds(0, 0); //The Zumo stops
  int counts = 0;
  //resets the encoders and the counts variable
  encoders.getCountsAndResetRight();

  //drive forward a small distance to center the robot over the line
  motors.setSpeeds(80, 80);
  while (counts < 450) {
    //900 cpr
    counts = encoders.getCountsRight();
    delay(5);
  }
  motors.setSpeeds(0, 0);
  while (turnLine == false) {
    motors.setSpeeds(100, -100);
    calibratedLineSensors();
    if (lineSensorValues[2] > threshHold) {
      turnLine = true;
    }
  }
  while (iAmHome == false) {
    turnLine = false;

    motors.setSpeeds(motorSpeed, motorSpeed);
    calibratedLineSensors();
    if (lineSensorValues[1] > threshHold) {
      motors.setSpeeds(50, 100);
      delay(10);

    }
    if (lineSensorValues[3] > threshHold) {
      motors.setSpeeds(100, 50);
      delay(10);
    }
    if (lineSensorValues[4] > threshHold) {
      counts = 0;
      //resets the encoders and the counts variable
      encoders.getCountsAndResetRight();

      //drive forward a small distance to center the robot over the line
      motors.setSpeeds(80, 80);
      while (counts < 50) {
        //900 cpr
        counts = encoders.getCountsRight();
        delay(5);
      }
        while (turnLine == false) {
          motors.setSpeeds(100, -100);
          calibratedLineSensors();
          if (lineSensorValues[2] > threshHold) {
            turnLine = true;
          }
        }
      
    }
    if (lineSensorValues[0] > threshHold) {
      motors.setSpeeds(0, 0);
      iAmHome = true;
    }
  }
}

//=========== end program ==========
