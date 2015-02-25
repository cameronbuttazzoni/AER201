#define SERVO_PIN 9

#include <Servo.h>
Servo myServo;
int angle;

void setup(){
  myServo.attach(SERVO_PIN);
  Serial.begin(9600);
}

void loop(){
  angle = 0;
  myServo.write(angle);
  delay(1000);
  angle = 60;
  myServo.write(angle);
  delay(1000);
}
