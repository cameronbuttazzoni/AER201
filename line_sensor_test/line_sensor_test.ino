#define LEFT_LINE_PIN 4
#define MID_LINE_PIN 5
#define RIGHT_LINE_PIN 6
#define LEFT_LED_PIN 2
#define MID_LED_PIN 3
#define RIGHT_LED_PIN 7
#define PULSE_DELAY 3

void setup(){
  //Serial.begin(9600);
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(MID_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);
  digitalWrite(LEFT_LED_PIN, LOW);
  digitalWrite(MID_LED_PIN, LOW);
  digitalWrite(RIGHT_LED_PIN, LOW);
  delay(1000);
}

void loop(){
  pinMode(LEFT_LINE_PIN, OUTPUT);
  pinMode(MID_LINE_PIN, OUTPUT);
  pinMode(RIGHT_LINE_PIN, OUTPUT);
  digitalWrite(LEFT_LINE_PIN, HIGH);
  digitalWrite(MID_LINE_PIN, HIGH);
  digitalWrite(RIGHT_LINE_PIN, HIGH);
  pinMode(LEFT_LINE_PIN, INPUT);
  pinMode(MID_LINE_PIN, INPUT);
  pinMode(RIGHT_LINE_PIN, INPUT);
  //delayMicroseconds(10);
  delay(PULSE_DELAY);
  int left_val = digitalRead(LEFT_LINE_PIN);
  int mid_val = digitalRead(MID_LINE_PIN);
  int right_val = digitalRead(RIGHT_LINE_PIN);
  if (left_val == HIGH) digitalWrite(LEFT_LED_PIN, HIGH);
  else digitalWrite(LEFT_LED_PIN, LOW);
  if (mid_val == HIGH) digitalWrite(MID_LED_PIN, HIGH);
  else digitalWrite(MID_LED_PIN, LOW);
  if (right_val == HIGH) digitalWrite(RIGHT_LED_PIN, HIGH);
  else digitalWrite(RIGHT_LED_PIN, LOW);
  /*Serial.print(left_val);
  Serial.print('\t');
  Serial.print(mid_val);
  Serial.print('\t');
  Serial.println(right_val);
  delay(50);*/
}
