#define LEFT_IR_PIN A4
#define RIGHT_IR_PIN A5

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
  last_val = analogRead(A0);
  if (last_val > 0){ last_val = 1;}
  //Serial.println(last_val);
}
/*void loop(){
  while (init_time + 20000 > cur_time){
  cur_val = analogRead(A0);
  if (cur_val > 0) cur_val = 1;
  if (cur_val != last_val){
    last_val = cur_val;
    //Serial.println(cur_val);
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
  Serial.println(cur_val);
  Serial.print('\t');
  Serial.println(cur_val2);
  delay(50);}
