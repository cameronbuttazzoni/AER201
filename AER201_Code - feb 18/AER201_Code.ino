/* AER201 ROBOT CODE
Team: 1 Plus 1 Plus 1 Plus 1 Plus Rob the Robot
Code By: Cameron Buttazzoni
Other Team Members: Yew Meng Khaw, Abhimanyu Joshi, James Tu

Version Control:
    1.0: 
        - Created initial code for the hopper detection, does not yet take into account robot movement, start/end time
            1.1:
	        - Added Pseudocode describing the planned implentation of the code; describes the necessary task of every function, and the order they are called in
            1.2:
                - Implemented hopper detection algorithm
            1.3:
                - Fixed hopper detection algroithm to require many readings in a row before accepting a value (to overcome sonar inaccuracies)
                      

Need to Change List:

Potential Problems List:

Need to Add List:
    - Start and end location for Hopper Detection
    - Movement in hopper Detection
    - As possible errors are found in function, update their corresponding error handler
    - Add emergency stop interrupt

*/
#include <NewPing.h>
#include <math.h>

//Defined Constants
#define SERIAL_COMM_BOOL 1 //1 if using serial communication, else 0
#define HOP_DETECT_TRIG_PIN 5 //Hopper detecting distance sensor's trigger pin
#define HOP_DETECT_ECHO_PIN 6 //Hopper detecting distance sensor's echo pin
#define HOP_DETECT_MAX_DIST 70 //Maximum distance values collected (in cm)
#define HOP_DETECT_MIN_DIST 5 //Minimum distance values polled (in cm), keep > 0
#define HOP_DETECT_PING_DELAY 50 //time between pings from hopper detector distance sensor (min 30)
#define HOPPER_DETECT_ERROR 3 // distance differences where program thinks it has detected change
#define HOPPER_DETECT_MAX_ERROR 25 // max difference between leg values that trigger difference
#define HOPPER_DETETCT_Y_ERROR 2 // distance that counts as the same value when comparing to y_robot
#define HOPPER_VALUES_IN_ROW 2 //Number of similar values in a row that need to be found

#define NUMBER_OF_RAN_HOPPERS 2 //Number of random hoppers to detect
#define NUMBER_OF_DEF_HOPPERS 2 //Number of default hoppers
#define NUMBER_OF_HOPPERS (NUMBER_OF_RAN_HOPPERS + NUMBER_OF_DEF_HOPPERS) //total number of hoppers
#define RANDOM_HOPPER_BALLS_NUM 7 //Number of balls stored in the random hoppers
#define DEFAULT_HOPPER_BALLS_NUM 4 //Number of balls stored in the default corner hoppers
#define NUMBER_BOARD_COLUMNS 7 //number of columns on the game board
#define NUMBER_BOARD_ROWS 6 //number of rows on the game board
#define SERIAL_BAUD_RATE 9600 //Serial bits per second

#define WHEEL_IR_DELAY 15 //time between checks for the wheel IR sensors
#define WHEEL_CIRCUMFERENCE 15 //circumference of the wheel (in cm)
#define WHEEL_NUM_HOLES 12 //number of holes in the wheel

#define RIGHT_WHEEL_ENABLE_PIN 4 //enable the right wheel motor
#define LEFT_WHEEL_ENABLE_PIN 10 //enable the left wheel motor
#define LEFT_FORWARD_PIN 8 //Set to high to move left wheel forward
#define LEFT_BACKWARD_PIN 9 //Set to high to move left wheel backward
#define RIGHT_FORWARD_PIN 3 //set to high to move right wheel forward
#define RIGHT_BACKWARD_PIN 2 //set to high to move right wheel backward

#define LINE_SEP_DIST 20 //Distance between black lines (in cm)

//Structures

typedef struct 
{
 int  x; // Stores the x coordinate of the hopper 0 is leftmost side, 1 increase per line
 int  y; // Stores the y coordinate of the hopper 0 is bottom, 1 increase per line
 int balls; //Stores number of balls remaining in hopper
 int orient; //Stores the orientation of the hopper
 // 0 is middle leg towards top, 1 is middle leg towards bot, 2 is the bottom lefts, 3 is bottom rights
} Hopper;  

//Global Variables
NewPing hopper_detector(HOP_DETECT_TRIG_PIN, HOP_DETECT_ECHO_PIN, HOP_DETECT_MAX_DIST); // Create NewPing object for hopper detection
unsigned long ping_time = 0;     // holds time of the next ping
unsigned long left_ir_time = 0;     // holds time of the next check of left wheel's IR sensor
int prev_ir_left = 0; //stores the last value measured by the left wheel IR
unsigned long right_ir_time = 0;     // holds time of the next check of right wheel's IR sensor
int prev_ir_right = 0; //stores the last value measured by the right wheel IR
float initial_orient_robot; // holds the initial orientation value recorded from the compass
float x_robot; // holds robot's current x-axis location
float y_robot; // holds robot's current y-axis location
float orient_robot; //holds robot's orientation
Hopper hoppers[NUMBER_OF_HOPPERS]; // Saves the four game field hopper structures
int gameboard[NUMBER_BOARD_COLUMNS][NUMBER_BOARD_ROWS] = {0}; // 0 = no ball, 1 = our ball, 2 = their ball, 3 = probably our ball, 4 = probably their ball
unsigned int next_ball_column; //holds the column to play the next ball into?
unsigned int next_hopper; //holds the next hopper to get a ball from

//Functions
void setup_pins();
int detect_hoppers(); // detect hopper routine. Goes to initial position to left of hoppers then moves right and scans
void detect_hoppers_error(int error);
void update_location(long cur_time);
void update_hoppers_location(); // Saves the locations and orientations of all the hoppers in the hoppers array
void compass_init(); // measure the initial compass starting position and save to initial_orient_robot
void hopper_detect_initial_pos(); // go to initial position to the left of the hoppers
void init_hoppers(); // set up hoppers array
int get_first_ball(); // move the robot in position to grab the first ball
void get_first_ball_error(int error);
int grab_ball(); // Use the vacuum pump to grab the ball from the hopper
int go_to_board(); // Move the robot to the left of the game board with the sensors in position to scan the board
void go_to_board_error(int error);
int pre_board_scan(); // scan the board and update all new balls as opponent played balls
void pre_board_scan_error(int error);
int game_strategy(); // based on the current game board state, updates next_ball_column to the column we will play ball into
void game_strategy_error(int error);
int orient_ball_release(); // move robot in front of column specified by next_ball_column
void orient_ball_release_error(int error);
int play_ball(); //release the ball into the game board
void play_ball_error(int error);
int post_board_scan(); //scan the board and update 1 new ball as ours, new balls > 1, there is uncertainty in which is ours
void post_board_scan_error(int error);
int go_to_hopper(); // Go to the hopper specified by next_hopper
void go_to_hopper_error(int error);
//handles all serial communication NOTE: print newline character after function call
void serial_comm(long cur_time, int prev_dist, int prev_dist2, int cur_hopper, int flag, int ping_dist, int count); 
void stop_robot_motion(); //turn off wheel motors
void start_robot_forward(); //turn on wheel forwards to move forward



void setup(){
  Serial.begin(SERIAL_BAUD_RATE); // open serial moniter to get feedback
  int error_check; // Check if any routines don't run as expected. Return 0 if Routine runs error free
  setup_pins();
  compass_init(); // Get the initial orientation of the robot in the gameboard  
  init_hoppers(); // create structures for the hoppers
  error_check = detect_hoppers(); // Call routine to find position and orientation of hoppers
  if (error_check != 0){ detect_hoppers_error(error_check);} // In event of failure, call backup routine
  error_check = get_first_ball(); // code for getting the first ball from a random hopper
  if (error_check != 0){ get_first_ball_error(error_check);} // In event of failure, call backup routine
  
}

void setup_pins(){
  // Setup all of the pins
  pinMode(LEFT_WHEEL_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_WHEEL_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  //Enable the wheels to sping
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
}

void loop(){
  int error_check; // Check if any routines don't run as expected. Return 0 if Routine runs error free
  error_check = go_to_board();
  if (error_check != 0){ go_to_board_error(error_check);}
  error_check = pre_board_scan();
  if (error_check != 0){ pre_board_scan_error(error_check);}
  error_check = game_strategy();
  if (error_check != 0){ game_strategy_error(error_check);}
  error_check = orient_ball_release();
  if (error_check != 0){ orient_ball_release_error(error_check);}
  error_check = play_ball(); // place the ball in the game board
  if (error_check != 0){ play_ball_error(error_check);} 
  error_check = go_to_board();
  if (error_check != 0){ go_to_board_error(error_check);}
  error_check = post_board_scan();
  if (error_check != 0){ post_board_scan_error(error_check);}
  error_check = go_to_hopper();
  if (error_check != 0){ go_to_hopper_error(error_check);}
  
}

/*int detect_hoppers(){
  ping_time = millis(); // initialize ping time
  hopper_detect_initial_pos(); // get to initial position for detecting hoppers
  int prev_dist = 0;
  while (1){ //Run until end is reached
    if (millis() >= ping_time) {   // check if we need to send a new ping
      ping_time +=  HOP_DETECT_PING_DELAY;      // set the time of the next ping
      hopper_detector.ping_timer(update_hoppers_location); // send out a ping and call the update_hoppers_location function to handle the result
    }
    // Move the robot forward
    // Check if distance sensor returns value
    // If distance sensor returns value, update hoppers
    // If both hoppers are found, then end the function
    // If the robot reaches the end of the path (past sensors) return error
    }
}*/

/*int detect_hoppers(){
  hopper_detect_initial_pos(); // get to initial position for detecting hoppers
  long cur_time = millis(); //Record the current robot time
  ping_time = cur_time; // initialize ping time
  if (left_ir_time == 0){
    left_ir_time = cur_time;}
  if (right_ir_time == 0){
    right_ir_time = cur_time;}
  int prev_dist = 0; // Saves last measured important distance
  int prev_dist2 = 0; // Saves last distance, prevents one random inaccurate reading from messing with function
  int cur_hopper = 0; //record current hopper we are finding, either 0 or 1
  int flag = 1; //1 if we are ready to detect a new hopper, 2 if we found 3rd leg, else 0
  int ping_dist = 0; // keeps track of the distance measured by the most recent ping
  int count = 0;
  while (1){ //Run until end is reached
    cur_time = millis();
    update_location(cur_time); // updates x_robot and y_robot
    if (cur_time > ping_time){ // send another ping
      unsigned int ping_record_time = hopper_detector.ping(); //measures time to receive ping
      ping_dist = ping_record_time / US_ROUNDTRIP_CM + (int) y_robot; // ping distance from gamefield bottom
      Serial.print(ping_dist); // print the ping distance to Serial
      Serial.print("    ");
      Serial.println((int) y_robot);
      ping_time += HOP_DETECT_PING_DELAY; // add delay before another ping is sent
      if (ping_dist > 100) continue;
    }
    else continue;
    if (abs(ping_dist - prev_dist2) > HOPPER_DETECT_ERROR){ //Previous two values are very different so continue
      prev_dist2 = ping_dist;
      count = 0;
      continue;
    }
    else count ++;
    if (flag == 0){ // Need to find 3rd leg of hopper
      if (hoppers[cur_hopper-1].orient == 0){ // middle leg is far 
        if (ping_dist < prev_dist - HOPPER_DETECT_ERROR && ping_dist - (int) y_robot > HOPPER_DETETCT_Y_ERROR && count == HOPPER_VALUES_IN_ROW) flag = 2; // found the 3rd leg
      }
      if (hoppers[cur_hopper-1].orient == 1){ // middle leg is closer
        if (ping_dist > prev_dist + HOPPER_DETECT_ERROR && count == HOPPER_VALUES_IN_ROW) flag = 2; // found the 3rd leg
      }
    }
    if (flag == 2 && ping_dist - y_robot < HOPPER_DETETCT_Y_ERROR && count == HOPPER_VALUES_IN_ROW){ // Past the hopper
      prev_dist = 0; //set prev_dist back to 0
      flag = 1; //ready to search for next hopper
    }
    if (ping_dist > HOP_DETECT_MIN_DIST + (int) y_robot && flag == 1 && count == HOPPER_VALUES_IN_ROW){
      if (prev_dist == 0){ // Found first leg of hopper
        prev_dist = ping_dist; // prev dist stores first leg of hopper distance
        prev_dist2 = ping_dist;
        count = 0;
        continue;
      }
      if (ping_dist > prev_dist + HOPPER_DETECT_ERROR){ // Hopper is Orientation 0
        hoppers[cur_hopper].x = (int) x_robot / LINE_SEP_DIST;
        hoppers[cur_hopper].y = prev_dist / LINE_SEP_DIST;
        hoppers[cur_hopper].orient = 0;
        hoppers[cur_hopper].balls = RANDOM_HOPPER_BALLS_NUM;
        cur_hopper++; //start looking for the next hopper
        flag = 0;
        prev_dist = ping_dist; //update prev_dist to distance to the middle leg
        count = 0;
      }
      if (ping_dist < prev_dist - HOPPER_DETECT_ERROR){ // Hopper is Orientation 1
        // Save state
        hoppers[cur_hopper].x = (int) x_robot / LINE_SEP_DIST;
        hoppers[cur_hopper].y = prev_dist / LINE_SEP_DIST;
        hoppers[cur_hopper].orient = 1;
        hoppers[cur_hopper].balls = RANDOM_HOPPER_BALLS_NUM;
        cur_hopper++; // start looking for the next hopper
        flag = 0;
        prev_dist = ping_dist; //update prev_dist to distance to the middle leg
        count = 0;
      }
    }
    // If both hoppers are found, then end the function
    if (cur_hopper >= NUMBER_OF_RAN_HOPPERS) break;
    prev_dist2 = ping_dist; //update new previous distance value
    // If the robot reaches the end of the path (past sensors) return error
    }
  display_hoppers(); //Shows data for each hopper on serial
  return 0;
}*/

int detect_hoppers(){
  hopper_detect_initial_pos(); // get to initial position for detecting hoppers
  long cur_time = millis(); //Record the current robot time
  ping_time = cur_time; // initialize ping time
  if (left_ir_time == 0){
    left_ir_time = cur_time;}
  if (right_ir_time == 0){
    right_ir_time = cur_time;}
  int prev_dist = 0; // Saves last measured important distance
  int cur_hopper = 0; //record current hopper we are finding, either 0 or 1
  int flag = 1; // 1 if we are ready to detect a new hopper, 2 if we found 3rd leg, else 0
  int ping_dist = 0; // keeps track of the distance measured by the most recent ping
  int prev_dist2 = 0; //keeps track of previous ping
  int count = 0; // keeps track of the number of similar distance pings in a row
  while (1){ //Run until end is reached
    cur_time = millis(); //update current time every loop
    update_location(cur_time); // updates x_robot and y_robot
    if (cur_time > ping_time){ // send another ping if enough time has passed
      unsigned int ping_record_time = hopper_detector.ping(); //measures time to receive ping
      ping_dist = ping_record_time / US_ROUNDTRIP_CM + (int) y_robot; // ping distance from gamefield bottom
      //Serial communcation
      serial_comm(cur_time, prev_dist, prev_dist2, cur_hopper, flag, ping_dist, count);
      ping_time += HOP_DETECT_PING_DELAY; // add delay before another ping is sent
      if (ping_dist > 100) continue; //prevent sonar malfunctions from affecting anything
    }
    else continue; //wait until time for new ping
    if (abs(ping_dist - prev_dist2) > HOPPER_DETECT_ERROR){ //Previous two values are very different so continue
      prev_dist2 = ping_dist; //update prev_dist2 to newer ping
      count = 0; //reset count to 0 since cycle of similar ping distances is broken
      continue; //wait for next ping
    }
    else count ++; //current ping distance is similar to previous ones
    if (flag == 0){ // Need to find 3rd leg of hopper
      if (hoppers[cur_hopper-1].orient == 0){ // middle leg is far 
        if (ping_dist < prev_dist - HOPPER_DETECT_ERROR && ping_dist - (int) y_robot > HOPPER_DETETCT_Y_ERROR && count >= HOPPER_VALUES_IN_ROW) flag = 2; // found the 3rd leg
      }
      if (hoppers[cur_hopper-1].orient == 1){ // middle leg is closer
        if (ping_dist > prev_dist + HOPPER_DETECT_ERROR && count >= HOPPER_VALUES_IN_ROW) flag = 2; // found the 3rd leg
      }
    }
    //if (flag == 2 && ping_dist - y_robot < HOPPER_DETECT_Y_ERROR && count == HOPPER_VALUES_IN_ROW){ // Past the hopper
      // WONT WORK SINCE DOESNT RETURN 0, SO NEW IF STATEMENT FIXES IT
    if (flag == 2 && abs(prev_dist - ping_dist) > HOPPER_DETECT_ERROR && count >= HOPPER_VALUES_IN_ROW){ // Past the hopper
      prev_dist = 0; //set prev_dist back to 0
      flag = 1; //ready to search for next hopper
      count = 0; //reset count to 0 since we have past the hopper
    }
    //if (ping_dist > HOP_DETECT_MIN_DIST + (int) y_robot && flag == 1 && count >= HOPPER_VALUES_IN_ROW){ // found a hopper
    if (ping_dist > HOP_DETECT_MIN_DIST + (int) y_robot){ // found a hopper
      if (prev_dist == 0){ // Found first leg of hopper
        prev_dist = ping_dist; // prev dist stores first leg of hopper distance
        prev_dist2 = ping_dist;
        count = 0;
        //stop_robot_motion();
        //delay(1000);
        //start_robot_forward();
        continue;
      }
      //found middle of hopper
      //if (ping_dist > prev_dist + HOPPER_DETECT_ERROR && ping_dist < prev_dist + HOPPER_DETECT_MAX_ERROR){ // Hopper is Orientation 0
      if (ping_dist > prev_dist + HOPPER_DETECT_ERROR){ // Hopper is Orientation 0
        hoppers[cur_hopper].x = (int) x_robot / LINE_SEP_DIST;
        hoppers[cur_hopper].y = prev_dist / LINE_SEP_DIST;
        hoppers[cur_hopper].orient = 0;
        hoppers[cur_hopper].balls = RANDOM_HOPPER_BALLS_NUM;
        cur_hopper++; //start looking for the next hopper
        flag = 0;
        prev_dist = ping_dist; //update prev_dist to distance to the middle leg
        count = 0; //found middle leg of hopper so reset count to 0
      }
      //if (ping_dist < prev_dist - HOPPER_DETECT_ERROR && ping_dist > prev_dist - HOPPER_DETECT_MAX_ERROR){ // Hopper is Orientation 1
      if (ping_dist < prev_dist - HOPPER_DETECT_ERROR){ // Hopper is Orientation 1
        // Save state
        hoppers[cur_hopper].x = (int) x_robot / LINE_SEP_DIST;
        hoppers[cur_hopper].y = prev_dist / LINE_SEP_DIST;
        hoppers[cur_hopper].orient = 1;
        hoppers[cur_hopper].balls = RANDOM_HOPPER_BALLS_NUM;
        cur_hopper++; //start looking for the next hopper
        flag = 0;
        prev_dist = ping_dist; //update prev_dist to distance to the middle leg
        count = 0; //found middle leg of hopper so reset count to 0
      }
    }
    // If both hoppers are found, then end the function
    if (cur_hopper >= NUMBER_OF_RAN_HOPPERS) break;
    prev_dist2 = ping_dist; //update new previous distance value
    // If the robot reaches the end of the path (past sensors) return error
    }
  display_hoppers(); //Shows data for each hopper on serial
  return 0;
}

/*int detect_hoppers(){
  hopper_detect_initial_pos(); // get to initial position for detecting hoppers
  long cur_time = millis(); //Record the current robot time
  ping_time = cur_time; // initialize ping time
  int ping_dist = 0; // keeps track of the distance measured by the most recent ping
  long next_time = cur_time + 5000;
  int flag = 0;
  while (1){ //Run until end is reached
    cur_time = millis();
    if (cur_time > ping_time){ // send another ping
      unsigned int ping_record_time = hopper_detector.ping(); //measures time to receive ping
      ping_dist = ping_record_time / US_ROUNDTRIP_CM + (int) y_robot; // ping distance from gamefield bottom
      Serial.print(ping_dist); // print the ping distance to Serial
      Serial.print("    ");
      Serial.println((int) y_robot);
      ping_time += HOP_DETECT_PING_DELAY; // add delay before another ping is sent
    }
    if (cur_time > next_time){
      if (flag == 0){
        flag = 1;
        digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
        digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
        next_time += 5000;
      }
      else{
        flag = 0;
        digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
        digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
        next_time += 5000;
      }
    }
  }
}*/

void update_location(long cur_time){
  if (cur_time > right_ir_time){
    x_robot += 0.1; // TEMP **CHANGE**
    //check IRs
    right_ir_time += WHEEL_IR_DELAY;
  }
}

void display_hoppers(){
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, LOW);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, LOW);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
  if (SERIAL_COMM_BOOL && 0){
    for (int x = 0; x < NUMBER_OF_HOPPERS; x++){
      Serial.print("Hopper ");
      Serial.print(x+1);
      Serial.println(":");
      Serial.print("X: ");
      Serial.print(hoppers[x].x);
      Serial.print(" Y:");
      Serial.print(hoppers[x].y);
      Serial.print(" Balls: ");
      Serial.print(hoppers[x].balls);
      Serial.print(" Orient: ");
      Serial.println(hoppers[x].orient);
    }
  }
}

void stop_robot_motion(){
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, LOW);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, LOW);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
}

void start_robot_forward(){
  digitalWrite(RIGHT_WHEEL_ENABLE_PIN, HIGH);
  digitalWrite(LEFT_WHEEL_ENABLE_PIN, HIGH);
  delay(10);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
}

void detect_hoppers_error(int error){
  // Set error flags so that robot assumes just one random hopper in middle, but don't drive through middle error
}

void update_hoppers_location(){ 
  if (hopper_detector.check_timer()) { // True if a distance ping was measured
    Serial.print("Ping: "); //Serial prints for checking values received - not in final code
    Serial.print(hopper_detector.ping_result / US_ROUNDTRIP_CM); //convert microsecond result of time for ping to return to cm using US_ROUNDTRIP_CM
    Serial.println("cm");
    // Compare received values to previous received values
    // two orientations of hoppers are (far - closer - far) and (closer - far - closer)
    // save hoppers to struct
  }
}

void compass_init(){
  // save the current compass orientation to global variables
}

void hopper_detect_initial_pos(){
  // drive the robot to a position on left side of board just outside of the starting area
  // update current location
  x_robot = 0.0;
  y_robot = 0.0;
  //drive forward
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
}

void init_hoppers(){
  memset(hoppers, 0, sizeof(Hopper) * NUMBER_OF_HOPPERS);
  hoppers[NUMBER_OF_RAN_HOPPERS].x = 0; //bottom left hopper
  hoppers[NUMBER_OF_RAN_HOPPERS].y = 0;
  hoppers[NUMBER_OF_RAN_HOPPERS].orient = 2;
  hoppers[NUMBER_OF_RAN_HOPPERS].balls = DEFAULT_HOPPER_BALLS_NUM;
  hoppers[NUMBER_OF_RAN_HOPPERS + 1].x = 8; //bottom right hopper
  hoppers[NUMBER_OF_RAN_HOPPERS + 1].y = 8;
  hoppers[NUMBER_OF_RAN_HOPPERS + 1].orient = 3;
  hoppers[NUMBER_OF_RAN_HOPPERS + 1].balls = DEFAULT_HOPPER_BALLS_NUM;
}
int get_first_ball(){
	// Follow path around outside of the board to the random hopper nearest the gameboard
	// Set this random hopper as the first one to clear
	return grab_ball();
	
}

int grab_ball(){
	// orient vacuum pump over hopper
	// turn on vacuum pump to grab ball
}
int go_to_board(){
	// Drive from a hopper to in front of the game board
	// Should be positioned on the left side of the board to start scan
}

void go_to_board_error(int error){
	// In event of error
}

int pre_board_scan(){
	for (int x = 0; x < NUMBER_BOARD_COLUMNS; x++){
		// Scan column of board and update the array
		// Move the robot to the next column
		// All new balls are played by the opponent
	}
}

void pre_board_scan_error(int error){
	// handle errors
}

int game_strategy(){
	// find best column to place the ball into
  // save to next_ball_column global variable
}

void game_strategy_error(int error){
	// handle errors
}

int orient_ball_release(){
	// move robot so ball can be released in column specified by next_ball_column
}

void orient_ball_release_error(int error){
	// handle errors
}

int play_ball(){
	//call actuator to release the ball allowing it to slide into the game board column
}

void play_ball_error(int error){
	// handle errors
}
int post_board_scan(){
	for (int x = 0; x < NUMBER_BOARD_COLUMNS; x++){
		// Scan column of board and update the array
		// Move the robot to the next column
	}
  // identify new balls as ours
	// if there is more than one new ball, guess which is ours and save with uncertainty
}

void post_board_scan_error(int error){
	// handle errors
}

int go_to_hopper(){
	// go to the hopper specified by next_hopper
}

void go_to_hopper_error(int error){
	// handle errors
}

void serial_comm(long cur_time, int prev_dist, int prev_dist2, int cur_hopper, int flag, int ping_dist, int count){
  if (SERIAL_COMM_BOOL){
    Serial.print(ping_time);
    Serial.print(',');
    Serial.print(left_ir_time);
    Serial.print(',');
    Serial.print(prev_ir_left);
    Serial.print(',');
    Serial.print(right_ir_time);
    Serial.print(',');
    Serial.print(prev_ir_right);
    Serial.print(',');
    Serial.print(initial_orient_robot);
    Serial.print(',');
    Serial.print(x_robot);
    Serial.print(',');
    Serial.print(y_robot);
    Serial.print(',');
    Serial.print(orient_robot);
    Serial.print(',');
    Serial.print(next_ball_column);
    Serial.print(',');
    Serial.print(millis());
    for (int x; x< NUMBER_OF_HOPPERS; x++){
      Serial.print(',');
      Serial.print(hoppers[x].x);
      Serial.print(',');
      Serial.print(hoppers[x].y);
      Serial.print(',');
      Serial.print(hoppers[x].balls);
      Serial.print(',');
      Serial.print(hoppers[x].orient);
    }
    Serial.print(',');
    Serial.print(cur_time);
    Serial.print(',');
    Serial.print(prev_dist);
    Serial.print(',');
    Serial.print(prev_dist2);
    Serial.print(',');
    Serial.print(cur_hopper);
    Serial.print(',');
    Serial.print(flag);
    Serial.print(',');
    Serial.print(ping_dist);
    Serial.print(',');
    Serial.println(count);
  }
}
