#define RIGHT_WHEEL_ENABLE_PIN 10 //enable the left wheel motor
#define LEFT_WHEEL_ENABLE_PIN 5 //enable the right wheel motor
#define LEFT_FORWARD_PIN 6 //Set to high to move right wheel forward
#define LEFT_BACKWARD_PIN 7 //Set to high to move right wheel backward
#define RIGHT_FORWARD_PIN 9 //set to high to move left wheel forward
#define RIGHT_BACKWARD_PIN 8 //set to high to move left wheel backward
#define LEFT_IR_PIN A4
#define RIGHT_IR_PIN A5

void setup_pins(){
  // Setup all of the pins
  pinMode(LEFT_WHEEL_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_WHEEL_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
}

void drive_stop(){
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, LOW);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, LOW);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
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

void drive_backward(){
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
  //analogWrite(LEFT_WHEEL_ENABLE_PIN, 100);
  digitalWrite(LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
}

void drive_left(){
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
}

void drive_right(){
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
}

int cur_val;
int cur_val2;
int last_val;
unsigned long init_time;
unsigned long cur_time;
unsigned long counter;
void setup(){
  Serial.begin(9600);
  init_time = millis();
  cur_time = millis();
  last_val = analogRead(LEFT_IR_PIN);
  if (last_val > 0){ last_val = 1;}
  setup_pins();
  drive_forward();
  Serial.println(last_val);
}
/*void loop(){
  while (init_time + 20000 > cur_time){
  cur_val = analogRead(IR_PIN);
  if (cur_val > 0) cur_val = 1;
  if (cur_val != last_val){
    last_val = cur_val;
    Serial.println(cur_val);
    if (cur_val == 1) {
    counter++;
    Serial.println(counter);}}
  cur_time = millis();
  delay(5);
  }
}*/

void loop(){
  cur_val = analogRead(LEFT_IR_PIN);
  cur_val2 = analogRead(RIGHT_IR_PIN);
  Serial.print(cur_val);
  Serial.print('\t');
  Serial.println(cur_val2);
  delay(50);}
