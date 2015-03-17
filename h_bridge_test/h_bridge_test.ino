#define RIGHT_WHEEL_ENABLE_PIN 6 //enable the left wheel motor
#define LEFT_WHEEL_ENABLE_PIN 5 //enable the right wheel motor
#define LEFT_FORWARD_PIN 10 //Set to high to move right wheel forward
#define LEFT_BACKWARD_PIN 9 //Set to high to move right wheel backward
#define RIGHT_FORWARD_PIN 13 //set to high to move left wheel forward
#define RIGHT_BACKWARD_PIN 12 //set to high to move left wheel backward
int right_speed = 255;
int left_speed = 255;
void setup(){
  setup_pins();
  drive_forward_analog();
}

void loop(){
  //First
  //drive_forward();
  //delay(2000);
  //Second
  /*
  //Third
  delay(2000);
  //Fourth
  delay(2000);*/
}

void setup_pins(){
  // Setup all of the pins
  pinMode(LEFT_WHEEL_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_WHEEL_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
}

void drive_forward(){
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
  //analogWrite(LEFT_WHEEL_ENABLE_PIN, 100);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
}

void drive_forward_analog(){
  analogWrite(RIGHT_WHEEL_ENABLE_PIN,right_speed);
  analogWrite(LEFT_WHEEL_ENABLE_PIN, left_speed);
  //analogWrite(LEFT_WHEEL_ENABLE_PIN, 100);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
}

void drive_backward(){
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
  //analogWrite(LEFT_WHEEL_ENABLE_PIN, 100);
  digitalWrite(LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
}

void drive_backward_analog(){
  analogWrite(RIGHT_WHEEL_ENABLE_PIN, right_speed);
  analogWrite(LEFT_WHEEL_ENABLE_PIN, left_speed);
  //analogWrite(LEFT_WHEEL_ENABLE_PIN, 100);
  digitalWrite(LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
}

void drive_clockwise(){
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
}

void drive_clockwise_analog(){
  analogWrite(RIGHT_WHEEL_ENABLE_PIN, right_speed);
  analogWrite(LEFT_WHEEL_ENABLE_PIN, left_speed);
  digitalWrite(LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
}

void drive_counterclockwise(){
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
}

void drive_counterclockwise_analog(){
  analogWrite(RIGHT_WHEEL_ENABLE_PIN, right_speed);
  analogWrite(LEFT_WHEEL_ENABLE_PIN, left_speed);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
}
