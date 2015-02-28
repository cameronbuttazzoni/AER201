#define LEFT_LINE_PIN 4
#define MID_LINE_PIN 5
#define RIGHT_LINE_PIN 6
#define LEFT_LED_PIN 2
#define MID_LED_PIN 3
#define RIGHT_LED_PIN 7
#define PULSE_DELAY 3

void setup(){
  Serial.begin(9600);
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
  delayMicroseconds(10);
  long cur_time = micros();
  int left_val = digitalRead(LEFT_LINE_PIN);
  int mid_val = digitalRead(MID_LINE_PIN);
  int right_val = digitalRead(RIGHT_LINE_PIN);
  int left_time = 0;
  int mid_time = 0;
  int right_time = 0;
  while (left_val == HIGH || mid_val == HIGH || right_val == HIGH){
    left_val = digitalRead(LEFT_LINE_PIN);
    mid_val = digitalRead(MID_LINE_PIN);
    right_val = digitalRead(RIGHT_LINE_PIN);
    if (left_val == LOW && left_time == 0) left_time = micros() - cur_time;
    if (mid_val == LOW && mid_time == 0) mid_time = micros() - cur_time;
    if (right_val == LOW && right_time == 0) right_time = micros() - cur_time;
    if (micros() - cur_time > 10000){
      left_time = 10000;
      mid_time = 10000;
      right_time = 10000;
      break;
    }
  }
  Serial.print(left_time);
  Serial.print('\t');
  Serial.print(mid_time);
  Serial.print('\t');
  Serial.println(right_time);
  delay(50);
}
