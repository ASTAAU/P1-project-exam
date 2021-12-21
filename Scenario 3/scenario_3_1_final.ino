#include <Zumo32U4.h>
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ButtonA buttonA;
int count1;
int count2;

void setup() {
Serial.begin(9600);


}

void loop() {
  buttonA.waitForButton();
  count1 = encoders.getCountsAndResetRight();
  Serial.println(count1);
  motors.setSpeeds(200,200);
  delay(60000);
  count2=encoders.getCountsRight();
  Serial.println(count2);
  
}
