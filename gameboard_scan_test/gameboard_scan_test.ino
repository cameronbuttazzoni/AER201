const int ball_vals[] = {15, 66, 15, 10, 60, 55};
int vals[6] = {0};
void setup(){
  Serial.begin(9600);
  /*for (int x = A1; x < A7; x++){
    pinMode(x, INPUT);
  }*/
}

void loop(){
  for (int x = A1; x < A7; x++){
    vals[x-A1] = analogRead(x);
    Serial.print(vals[x-A1]);
    if (vals[x-A1] < ball_vals[x-A1]) Serial.print('B');
    else Serial.print('E');
    Serial.print('\t');
  }
  Serial.println('\0');
  
  delay(50);
}
