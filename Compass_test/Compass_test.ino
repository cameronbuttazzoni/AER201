#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
#define OUR_PI 3.14159265358979323846
#define NUM 9

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  pinMode(13, OUTPUT);
}
float x_vals[NUM];
float y_vals[NUM];
float z_vals[NUM];
float headings[NUM];
int count = 0;
void loop(void) 
{
  /* Get a new sensor event */ 
 if (count < NUM){
   digitalWrite(13, HIGH);
   delay(5000);
   digitalWrite(13, LOW);
   delay(100);
   digitalWrite(13, HIGH);
   delay(300);
   digitalWrite(13, LOW);
   delay(100);
   digitalWrite(13, HIGH);
   delay(300);
   digitalWrite(13, LOW);
   delay(100);
   sensors_event_t event; 
    mag.getEvent(&event);
   x_vals[count] = event.magnetic.x;
   y_vals[count] = event.magnetic.y;
   z_vals[count] = event.magnetic.z;
   float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.1745;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*OUR_PI;
  if(heading > 2*OUR_PI)
    heading -= 2*OUR_PI;
  float headingDegrees = heading * 180/OUR_PI; 
  headings[count] = headingDegrees;
  count++;
   delay(100);}
  else{
    for (int h = 0; h < NUM; h++){ Serial.print("X: "); Serial.print(x_vals[h]); Serial.print(" Y: "); Serial.print(y_vals[h]); Serial.print(" Z: "); Serial.print(z_vals[h]); Serial.print(" H: "); Serial.println(headings[h]);}
    while(1);}
  delay(100);
}
