#define MIN_PIN A0
#define MAX_PIN A5

void setup(){
  for (int x = MIN_PIN; x <= MAX_PIN; x++){
    pinMode(x, INPUT);
  }
  Serial.begin(9600);
}

void loop(){
  for (int x = MIN_PIN; x <= MAX_PIN; x++){
    int val = analogRead(x);
    Serial.print(val);
    Serial.print('\t');
  }
  Serial.println('\0');
  delay(50);
}
