//Code for the QRE1113 Digital board
//Outputs via the serial terminal - Lower numbers mean more reflected
//3000 or more means nothing was reflected.

int pin1 = 6; //connected to digital pin
int pin2 = 7;
int pin3 = 8;

void setup(){
  Serial.begin(9600);
}


void loop(){

  int val1 = readQD1();
  int val2 = readQD2();
  int val3 = readQD3();
  Serial.print(val1); 
  Serial.print('\t'); 
  Serial.print(val2); 
  Serial.print('\t'); 
  Serial.println(val3); 
  delay(40);

}


int readQD1(){
  //Returns value from the QRE1113 
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode( pin1, OUTPUT );
  digitalWrite( pin1, HIGH );  
  delayMicroseconds(10);
  pinMode( pin1, INPUT );

  long time = micros();

  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(pin1) == HIGH && micros() - time < 3000); 
  int diff = micros() - time;

  return diff;
}

int readQD2(){
  //Returns value from the QRE1113 
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode( pin2, OUTPUT );
  digitalWrite( pin2, HIGH );  
  delayMicroseconds(10);
  pinMode( pin2, INPUT );

  long time = micros();

  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(pin2) == HIGH && micros() - time < 3000); 
  int diff = micros() - time;

  return diff;
}

int readQD3(){
  //Returns value from the QRE1113 
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode( pin3, OUTPUT );
  digitalWrite( pin3, HIGH );  
  delayMicroseconds(10);
  pinMode( pin3, INPUT );

  long time = micros();

  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(pin3) == HIGH && micros() - time < 3000); 
  int diff = micros() - time;

  return diff;
}
