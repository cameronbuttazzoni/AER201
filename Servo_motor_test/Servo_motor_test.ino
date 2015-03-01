#define SERVO_PIN 11


#include <Servo.h>
Servo myServo;
int angle;

void setup(){
  myServo.attach(SERVO_PIN);
  Serial.begin(9600);
}

/*void loop(){
  int count = 160;
  int flag = 1;
  while (1){
    myServo.write(count);
    if (flag == 0) count += 5;
    else count -= 5;
    delay(1000);
    Serial.println(count);
    if (count == 160) flag = 1;
    if (count == 130) flag = 0;
  }
  angle = 199;
  myServo.write(angle);
  delay(1000);
  angle = 199;
  myServo.write(angle);
  delay(5000);
}*/

void loop(){
  myServo.write(170);
  delay(3000);
  //myServo.write(135);
  delay(5000);
}
