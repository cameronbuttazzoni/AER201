#define IN_PIN 7
#define DELAY_VAL 50

void setup(){
  pinMode(IN_PIN);
  Serial.begin(9600);
}
void loop(){
  int val = digitalRead(IN_PIN);
  Serial.println(val);
  delay(DELAY_VAL);
}
