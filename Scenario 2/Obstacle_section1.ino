  //-----Includes-----
  #include <Wire.h>
  #include <Zumo32U4.h>
  #include "TurnSensor.h"
  
  // ----- Robot components-------- (de ting vi bruger på robotten)
  Zumo32U4LCD lcd;
  Zumo32U4Motors motors;
  Zumo32U4IMU imu;
  Zumo32U4ProximitySensors proxSensors;
  LSM303 lsm303; //accelerameter 
  L3G gyro;
  Zumo32U4Encoders encoders;
  Zumo32U4ButtonA buttonA;
  Zumo32U4ButtonB buttonB;
  Zumo32U4ButtonC buttonC;
  Zumo32U4Buzzer buzz;
  
  //------Konstanter (hastighed)------
  const int16_t motorSpeed = 100;
  const int16_t turnSpeed  = 100;
  int distanceDriven = 0; 
  bool freeBritney = false; 
  int brightnessLevel[7] = {0.5, 1, 1.5, 2, 2.5, 3, 3.5};
  int brightnessLevelSide [7] = {2, 4, 6, 8, 10, 12, 14};
  int waitTime = 600;
  
  //---------Support functions-------
  //Turn left
  void turnLeft(int degrees) {
    turnSensorReset();
    motors.setSpeeds(-turnSpeed, turnSpeed);
    int angle = 0;
    do {
      delay(1);
      turnSensorUpdate();
      angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
      lcd.gotoXY(0, 0);
      lcd.print(angle);
      lcd.print(" ");
    } while (angle < degrees);
    motors.setSpeeds(0, 0);
  }
  
  //Turn right
  void turnRight(int degrees) {
    turnSensorReset();
    motors.setSpeeds(turnSpeed, -turnSpeed);
    int angle = 0;
    do {
      delay(1);
      turnSensorUpdate();
      angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
      lcd.gotoXY(0, 0);
      lcd.print(angle);
      lcd.print(" ");
    } while (angle > -degrees);
    motors.setSpeeds(0, 0);
  }

//Front sensor detects
  bool detectFront() {
   proxSensors.setBrightnessLevels(brightnessLevel, 6);
    proxSensors.read();
    uint8_t sumOfFront = proxSensors.countsFrontWithRightLeds() + proxSensors.countsFrontWithLeftLeds();
    Serial.println("Total sum = " + (String)sumOfFront);
    delay(20);
  
    if (sumOfFront >= 12) {
      return true;
    }
    else {
      return false;
    }
  }
 //Right sensor detects 
    bool detectRightWall() {
  proxSensors.setBrightnessLevels(brightnessLevelSide, 6);
      proxSensors.read();
      uint8_t sumOfRight = proxSensors.countsRightWithRightLeds();
      Serial.println("total right counts " + (String)sumOfRight);
      delay(20);
      if (sumOfRight >= 3) {
        return true;
      }
      else {
        return false;
      }
    }
  
 //Left sensor detects
  bool detectLeftWall() {
    proxSensors.setBrightnessLevels(brightnessLevelSide, 6);
        proxSensors.read();
        uint8_t sumOfLeft = proxSensors.countsLeftWithLeftLeds();
        Serial.println("total Left counts " + (String)sumOfLeft);
        delay(20);
        if (sumOfLeft >= 3) {
          return true;
        }
        else {
          return false;
        }
  
      }
//  -----------------------------The Code-----------------------------------------
  
  void setup(){
  Serial.begin(9600);
  proxSensors.initThreeSensors();  //Starte proximity sensors 
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  }
  
  
  void loop(){
  distanceDriven = 0;
  
  /* Læs data fra proximity sensor
    proxSensores.read();         
    int left_sensor        = proxSensors.countsLeftWithLeftLeds();
    int centerLeft_sensor  = proxSensors.countsFrontWithLeftLeds();
    int right_sensor       = proxSensors.countsLeftWithRightLeds();
    int centerRight_sensor = proxSensors.countsFrontWithRightLeds();
  */
  
    motors.setSpeeds(motorSpeed,motorSpeed);          //Kør fremad. motorSpeed --> en for hver hjul.
  
    if(detectFront()){                            //Hvis front sensor læser noget, så drejer den 90 grader til højre.
      delay(200);
      turnRight(90);
        encoders.getCountsAndResetLeft();
        while(detectLeftWall()){         //Så længe Zumo ser noget på venstre sensor køre den lige ud. Ellers hopper den ud af loopet, og begynder forfra med at køre lige ud
          motors.setSpeeds(motorSpeed,motorSpeed);
          delay(20);
        }
        
        
        delay(waitTime);
        distanceDriven = distanceDriven + encoders.getCountsAndResetLeft();         //Den tæller hvor langt Zumoen køre i dette while loop og tilføjer dette tal til vores 'int distanceDriven' og resetter
        turnLeft(90); //Dette er en function
        
      while(!detectLeftWall()){
         motors.setSpeeds(motorSpeed,motorSpeed);
      }
        delay(100);                                  //Delayet bliver brugt til at sige til Zumoen, hvor lang tid den skal køre fremad. Den stopper nemlig ikke, medmindre man giver den en funktion der siger den skal stoppe.
                        
        while(freeBritney == false){                  //While loopet køre så længe vores 'bool freeBritney' er false. Dvs. så længe enten front eller left sensor ser noget.
          
          if(detectFront()){
            delay(200);
            turnRight(90);
            encoders.getCountsAndResetLeft();
            
            while(detectLeftWall()){         //Så længe Zumo ser noget på venstre sensor køre den lige ud. Ellers hopper den ud af loopet, og begynder forfra med at køre lige ud
               motors.setSpeeds(motorSpeed,motorSpeed);
               delay(20);
        }
        
        
        delay(waitTime);
        distanceDriven = distanceDriven + encoders.getCountsAndResetLeft();         //Den tæller hvor langt Zumoen køre i dette while loop og tilføjer dette tal til vores 'int distanceDriven' og resetter
        turnLeft(90); //Dette er en function
            
            
            /*delay(200);
            turnRight(90);

              encoders.getCountsAndResetLeft();
              while(detectLeftWall()){         //Så længe Zumo ser noget på venstre sensor køre den, og gemmer/tilføjer denne kørte distance til vores 'int distanceDriven'.
                 motors.setSpeeds(motorSpeed,motorSpeed);
                 delay(20);
                 
              } 
              distanceDriven = distanceDriven + encoders.getCountsAndResetLeft();
             turnLeft(90);
             while(!detectLeftWall()){
               motors.setSpeeds(motorSpeed,motorSpeed);
              
             }
             delay(100); */   
           }else if(detectLeftWall()){
             motors.setSpeeds(motorSpeed,motorSpeed);
              
           }else{
            freeBritney = true;          //bool som gør at vi ændre værdi og gør at vi kommer ud af while loopet  
             buzz.playFrequency(440,100,10); 
             delay(waitTime); 
           }
           
          }
          turnLeft(90);
          encoders.getCountsAndResetLeft();
          while(encoders.getCountsLeft()<= distanceDriven){
          motors.setSpeeds(motorSpeed,motorSpeed);
          }
          
          turnRight(90);
          
      }
    
  }
