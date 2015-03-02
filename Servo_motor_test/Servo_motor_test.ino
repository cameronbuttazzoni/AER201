#define SERVO_PIN 11


#include <Servo.h>
Servo myServo;
int angle;

void setup(){
  myServo.attach(SERVO_PIN);
  Serial.begin(9600);
}

/*void loop(){
  int count = 0;
  int flag = 0; //0 counts up, 1 counts down
  while (1){
    myServo.write(count);
    if (flag == 0) count += 10;
    else count -= 10;
    delay(3000);
    Serial.println(count);
    if (count == 180) flag = 1;
    if (count == 0) flag = 0;
  }
  angle = 199;
  myServo.write(angle);
  delay(1000);
  angle = 199;
  myServo.write(angle);
  delay(5000);
}*/

void loop(){
  myServo.write(130);
  delay(2000);
  myServo.write(100);
  delay(2000);
}
