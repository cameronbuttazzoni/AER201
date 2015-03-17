//Code for the QRE1113 Digital board
//Outputs via the serial terminal - Lower numbers mean more reflected
//3000 or more means nothing was reflected.

int left_pin = 8; //connected to digital pin
int mid_pin = 7;
int right_pin = 3;
int main_pin = 2;
int side_pin = 11;

void setup(){
  Serial.begin(9600);
}


void loop(){

  int left_pin = readQDleft();
  int mid_pin = readQDmid();
  int right_pin = readQDright();
  int main_pin = readQDmain();
  int side_pin = readQDside();
  Serial.print(left_pin); 
  Serial.print('\t'); 
  Serial.print(mid_pin); 
  Serial.print('\t'); 
  Serial.print(right_pin); 
  Serial.print('\t'); 
  Serial.print(main_pin); 
  Serial.print('\t'); 
  Serial.println(side_pin); 
  delay(80);

}


int readQDleft(){
  //Returns value from the QRE1113 
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode(left_pin, OUTPUT );
  digitalWrite(left_pin, HIGH );  
  delayMicroseconds(10);
  pinMode(left_pin, INPUT );

  long time = micros();

  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(left_pin) == HIGH && micros() - time < 3000); 
  int diff = micros() - time;

  return diff;
}

int readQDmid(){
  //Returns value from the QRE1113 
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode(mid_pin, OUTPUT );
  digitalWrite(mid_pin, HIGH );  
  delayMicroseconds(10);
  pinMode(mid_pin, INPUT );

  long time = micros();

  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(mid_pin) == HIGH && micros() - time < 3000); 
  int diff = micros() - time;

  return diff;
}

int readQDright(){
  //Returns value from the QRE1113 
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode(right_pin, OUTPUT );
  digitalWrite(right_pin, HIGH );  
  delayMicroseconds(10);
  pinMode(right_pin, INPUT );

  long time = micros();

  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(right_pin) == HIGH && micros() - time < 3000); 
  int diff = micros() - time;

  return diff;
}

int readQDmain(){
  //Returns value from the QRE1113 
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode(main_pin, OUTPUT );
  digitalWrite(main_pin, HIGH );  
  delayMicroseconds(10);
  pinMode(main_pin, INPUT );

  long time = micros();

  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(main_pin) == HIGH && micros() - time < 3000); 
  int diff = micros() - time;

  return diff;
}

int readQDside(){
  //Returns value from the QRE1113 
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode(side_pin, OUTPUT );
  digitalWrite(side_pin, HIGH );  
  delayMicroseconds(10);
  pinMode(side_pin, INPUT );

  long time = micros();

  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(side_pin) == HIGH && micros() - time < 3000); 
  int diff = micros() - time;

  return diff;
}
