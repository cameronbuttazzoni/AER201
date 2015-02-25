#define RIGHT_WHEEL_ENABLE_PIN 4 //enable the left wheel motor
#define LEFT_WHEEL_ENABLE_PIN 10 //enable the right wheel motor
#define LEFT_FORWARD_PIN 8 //Set to high to move right wheel forward
#define LEFT_BACKWARD_PIN 9 //Set to high to move right wheel backward
#define RIGHT_FORWARD_PIN 3 //set to high to move left wheel forward
#define RIGHT_BACKWARD_PIN 2 //set to high to move left wheel backward
void setup(){
  setup_pins();
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
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

void drive_left(){
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
}

void loop(){
  //First
  drive_forward();
  delay(2000);
  //Second
  /*
  //Third
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
  delay(2000);
  //Fourth
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
  delay(2000);*/
}
