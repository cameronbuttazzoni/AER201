long count = 0;
long x = 0;
void setup(){
 Serial.begin(9600);
}

void loop(){
  long y1 = x * 3;
  long y2 = count * 4;
  long y3 = x * 2 + count;
  long y4 = count * x;
  Serial.print(count);
  Serial.print(',');
  Serial.print(x);
  Serial.print(',');
  Serial.print(y1);
  Serial.print(',');
  Serial.print(y2);
  Serial.print(',');
  Serial.print(y3);
  Serial.print(',');
  Serial.println(y4);
  count += 3;
  x++;
  delay(25);
}
