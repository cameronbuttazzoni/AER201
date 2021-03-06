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
            1.4:
                - Created function templates for the main program and the 8 tasks needed for progress evaluation 2
                - Started writing function for encoder to know robot orientation and direction
            1.5:
                - Finished wheel IR forward/backward/turning function (not tested)
                - Started adding black/white sensor for line detection code
            1.6:
                - Changed all cur_time recording things to microseconds
                - Implemented line following algorithm
            1.7:
                - Implemented moving to any coordinate on the gameboard, going to any line and 90 degree turns, aligning with gameboard, placing gameball, picking up gameball
            1.8:
                - Organized code into sections
                - Changed number of line sensors from 3 to 5
                      

Need to Change List:

Potential Problems List:
    - Only uses one wheel's IR sensor, so assumes both wheels spin at a constant speed

Need to Add List:
    - Start and end location for Hopper Detection
    - Movement in hopper Detection
    - As possible errors are found in function, update their corresponding error handler
    - Add emergency stop interrupt

*/
#include <NewPing.h>
#include <Servo.h>
//#include <math.h>

//Defined Constants
// HOPPER DETECTION
#define SERIAL_COMM_BOOL 0 //1 if using serial communication, else 0
#define HOP_DETECT_TRIG_PIN 5 //Hopper detecting distance sensor's trigger pin
#define HOP_DETECT_ECHO_PIN 6 //Hopper detecting distance sensor's echo pin
#define HOP_DETECT_MAX_DIST 70 //Maximum distance values collected (in cm)
#define HOP_DETECT_MIN_DIST 5 //Minimum distance values polled (in cm), keep > 0
#define HOP_DETECT_PING_DELAY 50000 //time between pings from hopper detector distance sensor (min 30000) microseconds
#define HOPPER_DETECT_ERROR 3 // distance differences where program thinks it has detected change
#define HOPPER_DETECT_MAX_ERROR 25 // max difference between leg values that trigger difference
#define HOPPER_DETETCT_Y_ERROR 2 // distance that counts as the same value when comparing to y_robot
#define HOPPER_VALUES_IN_ROW 2 //Number of similar values in a row that need to be found

// GAMEFIELD CONSTANTS
#define NUMBER_OF_RAN_HOPPERS 2 //Number of random hoppers to detect
#define NUMBER_OF_DEF_HOPPERS 2 //Number of default hoppers
#define NUMBER_OF_HOPPERS (NUMBER_OF_RAN_HOPPERS + NUMBER_OF_DEF_HOPPERS) //total number of hoppers
#define RANDOM_HOPPER_BALLS_NUM 7 //Number of balls stored in the random hoppers
#define DEFAULT_HOPPER_BALLS_NUM 4 //Number of balls stored in the default corner hoppers
#define NUMBER_BOARD_COLUMNS 7 //number of columns on the game board
#define NUMBER_BOARD_ROWS 6 //number of rows on the game board
#define LINE_SEP_DIST 20 //Distance between black lines (in cm)
#define GAME_FIELD_WIDTH 160 //width of game_field in cm
#define GAME_FIELD_HEIGHT 180 //height of game field in cm
#define COLUMN_SEP_DIST 5 //distance from one gameboard column to the next NOT EXACT YET

// WHEEL CONSTANTS
#define LEFT_IR_PIN A5 //analog pin for the left wheel's IR sensor
#define RIGHT_IR_PIN 5 //analog pin for the right wheel's IR sensor
#define WHEEL_IR_DELAY 15000 //time between checks for the wheel IR sensors MICROSECONDS
#define WHEEL_CIRCUMFERENCE 15 //circumference of the wheel (in cm)
#define WHEEL_NUM_HOLES 12 //number of holes in the wheel

// ROBOT DIMENSIONS
#define ROBOT_WIDTH 10 //half the robot width in cm
#define ROBOT_MAX_WIDTH 15 //half the maximum robot width in cm
#define ROBOT_LENGTH_FRONT 10//distance from point of rotation to front of robot
#define ROBOT_LENGTH_BACK 5 //distance from point of rotation to back of robot
#define ROBOT_TURNING_RADIUS 5 //distance from the center of rotation to the wheels of the robot
#define ROBOT_BALL_DROP_X_DIST 5 //the number of cm the ball is from the wheel axis in direction perpendicular to wheel axis
#define ROBOT_BALL_DROP_Y_DIST 5 //the number of cm the ball is from the wheel axis in direction parallel to wheel axis

// MOTOR CONSTANTS
#define RIGHT_WHEEL_ENABLE_PIN 10 //enable the right wheel motor
#define LEFT_WHEEL_ENABLE_PIN 5 //enable the left wheel motor
#define LEFT_FORWARD_PIN 6 //Set to high to move left wheel forward
#define LEFT_BACKWARD_PIN 7 //Set to high to move left wheel backward
#define RIGHT_FORWARD_PIN 9 //set to high to move right wheel forward
#define RIGHT_BACKWARD_PIN 8 //set to high to move right wheel backward
#define LEFT_WHEEL_MAX_SPEED 255 //max of 255
#define RIGHT_WHEEL_MAX_SPEED 255 //max of 255
#define LEFT_WHEEL_TURN_SPEED 255 //max of 255
#define RIGHT_WHEEL_TURN_SPEED 255 //max of 255
#define LEFT_WHEEL_GAMEBOARD_SPEED 255
#define RIGHT_WHEEL_GAMEBOARD_SPEED 255

// LINE SENSORS
#define NUM_LINE_SENSORS 5 //number of line sensors
#define LEFT_LINE_SENSOR_PIN 2 //left line detecting black/white IR sensor pin
#define MID_LINE_SENSOR_PIN 3 //middle line detecting black/white IR sensor pin
#define RIGHT_LINE_SENSOR_PIN 4 //right line detecting black/white IR sensor pin
#define MAIN_LINE_SENSOR_PIN 0 //the line sensor keeping track of in between the wheels, at very center of robot
#define SIDE_LINE_SENSOR_PIN 0 //line sensor on RIGHT/LEFT side of robot
#define CHECK_LINE_SENSOR_TIME 450 //MICROSECONDS since the line sensors start to check for black/white use 750
#define PULSE_LINE_SENSOR_TIME 100000 //send a pulse every CHECK_LINE_SENSOR_TIME + PULSE_LINE_SENSOR_TIME MICROSECONDS
#define LINE_SENSOR_DELAY 10 //delay in milliseconds between checks to the line sensors NOT USED
#define LINE_PASS_ANGLE_ERROR 0.5//angle in radians must be within for passing lines to register position updates
#define LINE_POSITION_ERROR 5 //error in position when going over line in cm
#define GOOD_ON_TRACK_TIME 2000000 //microseconds that the robot should be on the line for
#define MIN_SPEED_CORRECTION_VALUE 10 //minimum value to change wheel speed by when veering off course
#define MAX_SPEED_CORRECTION_VALUE 40 //max value to change wheel speed for slight off course
#define ROBOT_TURN_UP_ERROR 0.05 //radians, make small

// BALL RELEASE AND PICKUP CONSTANTS
#define BALL_GRAB_FAN_PIN 4 //pin that controls the fan to suck up balls
#define BALL_RELEASE_SERVO_PIN 6 //pin that controls the servo motor to release the ball
#define BALL_SUCTION_ON_TIME 5000 //amount of time the vacuum sucks for when picking up a ball MILLISECONDS
#define BALL_SUCTION_OFF_TIME 5000 //amount of time the robot delays for after turning off the fan MILLISECONDS
#define BALL_RELEASE_SERVO_INITIAL_VAL 136 //value for when the servo is closed
#define BALL_RELEASE_SERVO_FINAL_VAL 102 //value for when the servo is open
#define BALL_RELEASE_POSITION_ERROR 2 //in cm if the robot is within this distance of the column it will stop to place a ball
#define RELEASE_BALL_RESET_DELAY 2000 //delay time to reset the servo after placing a ball in MILLISECONDS
#define RELEASE_BALL_Y_VALUE 165 //y_robot for when the robot is aligned to the game board
#define RELEASE_BALL_X_VALUE 50 //should be greater than 40

// OTHER CONSTANTS
#define GAME_BOARD_LEFT_Y 8 //when navigating to the gameboard, go to this Y-coordinate if on left half of board
#define GAME_BOARD_LEFT_X 2 //when navigating to the gameboard, go to this X-coordinate if on left half of board
#define GAME_BOARD_RIGHT_Y 8 //when navigating to the gameboard, go to this Y-coordinate if on right half of board
#define GAME_BOARD_RIGHT_X 6 //when navigating to the gameboard, go to this X-coordinate if on left half of board
#define SERIAL_BAUD_RATE 9600 //Serial bits per second

// MATH
#define PI_OVER_TWO 1.5708
#define PI_THREE_OVER_TWO 4.7124

//Test specific defines 

//1 Controlled Locomotion
#define TEST_ONE_INITIAL_X 0 //coordinates based on inital line start
#define TEST_ONE_INITIAL_Y 0
#define TEST_ONE_FINAL_X 0
#define TEST_ONE_FINAL_Y 0
#define TEST_ONE_INITIAL_ORIENT 0//radians

//2 Picking up a gameball
#define TEST_TWO_DELAY_TIME 5000000 //turn off fan after this time in microseconds 

//3 Placing a gameball
#define TEST_THREE_INITIAL_X 65
#define TEST_THREE_INITIAL_Y 8 
#define TEST_THREE_INITIAL_ORIENT PI_THREE_OVER_TWO //make PI_OVER_TWO
#define TEST_THREE_FINAL_COLUMN 6 //from 0 to 6

//4 Navigating the gameboard
#define TEST_FOUR_INITIAL_X 0 //on which line
#define TEST_FOUR_INITIAL_Y 0 //on which line
#define TEST_FOUR_INITIAL_ORIENT 0 //initial direction

//7 Locating an obstacle
#define TEST_SEVEN_INITIAL_X 2
#define TEST_SEVEN_INITIAL_Y 2
#define TEST_SEVEN_INITIAL_ORIENT PI_OVER_TWO
#define TEST_SEVEN_NUM_HOPPERS 1


//Structures

typedef struct 
{
 int  x; // Stores the x coordinate of the hopper 0 is leftmost side, 1 increase per line
 int  y; // Stores the y coordinate of the hopper 0 is bottom, 1 increase per line
 int balls; //Stores number of balls remaining in hopper
 int orient; //Stores the orientation of the hopper
 // 0 is middle leg towards top, 1 is middle leg towards bot, 2 is the bottom lefts, 3 is bottom rights
} Hopper;  

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        GLOBAL VARIABLES
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int column_locations[] = {65, 70, 75, 80, 85, 90, 95}; //stores the x-coordinates of each of the gameboard columns WRONG
const int line_sensor_order[] = {2, 3, 1, 4, 5}; //1 is left sensor, 2 is middle sensor, 3 is right sensor, 4 is main sensor, 5 is side sensor
const int line_sensor_delay[] = {700, 100, 250, 0, 0}; //cumulative delay, must be in order from smallest to largest 
int line_sensor_checks = 0;
int left_line_sensor_val = HIGH; //last measured value of each line sensor
int mid_line_sensor_val = HIGH;
int right_line_sensor_val = HIGH;
int main_line_sensor_val = HIGH;
int side_line_sensor_val = HIGH;
NewPing hopper_detector(HOP_DETECT_TRIG_PIN, HOP_DETECT_ECHO_PIN, HOP_DETECT_MAX_DIST); // Create NewPing object for hopper detection
unsigned long ping_time = 0;     // holds time of the next ping
unsigned long wheel_ir_time = 0;     // holds time of the next check of wheel IR sensors
unsigned long line_sensor_time = 0; //holds time of the next check of the line sensors
int line_sensor_state = 0; //if 0 need to turn on line sensor reflect, if 1 need to check for colour
int prev_ir_left = 0; //stores the last value measured by the left wheel IR
int prev_ir_right = 0; //stores the last value measured by the right wheel IR
float initial_orient_robot; // holds the initial orientation value recorded from the compass
int robot_direction; //1 if moving forward, -1 if moving backward, 2 if turning clockwise, -2 counterclockwise
float x_robot; // holds robot's current x-axis location
float y_robot; // holds robot's current y-axis location
int x_line_robot; //the number of the line horizontally that the robot has crossed, left is 0
int y_line_robot; //the number of the line vertically that the robot has crossed, bottom is 0
float robot_orient = 0; //holds robot's orientation, 0 is towards gameboard, clockwise is positive in radian
Hopper hoppers[NUMBER_OF_HOPPERS]; // Saves the four game field hopper structures
int gameboard[NUMBER_BOARD_COLUMNS][NUMBER_BOARD_ROWS] = {0}; // 0 = no ball, 1 = our ball, 2 = their ball, 3 = probably our ball, 4 = probably their ball
unsigned int next_ball_column; //holds the column to play the next ball into?
unsigned int next_hopper; //holds the next hopper to get a ball from
int left_wheel_speed = 255; //speed of left wheel
int right_wheel_speed = 255; //speed of right wheel
int correction_state = 0; //0 = no correction, 1 correct when "slightly off course", 2 correct when "far off course", pos is robot going right, neg is left (so correct opposite this)
int correction_factor = 0; //Indicates the degree of line correction the robot has experienced
int off_track_flag = 0; //has value of 1 if last line check was off the line
unsigned long on_track_time = 0; //records the last time the robot was on track
Servo fan_servo;
int robot_on_line = 1; //keeps track if the robot was on line from the last poll, 1 if was on line, else 0

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup_pins();
int detect_hoppers(); // detect hopper routine. Goes to initial position to left of hoppers then moves right and scans
void detect_hoppers_error(int error);
void update_location(unsigned long cur_time);
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
void serial_comm(unsigned long cur_time, int prev_dist, int prev_dist2, int cur_hopper, int flag, int ping_dist, int count); 
void stop_robot_motion(); //turn off wheel motors
void start_robot_forward(); //turn on wheels to move forward
void start_robot_straight_forward(); //drive full speed straight forward
void start_robot_backward(); //turn on wheels to move backwards
void start_robot_straight_backward(); //drive full speed straight backwards
void start_robot_clockwise(); //turn on wheels to turn clockwise
void start_robot_counterclockwise(); //turn on wheels to turn counterclockwise
void robot_quarter_turn_clockwise(); //turns clockwise until the line sensors all match on a line
void robot_quarter_turn_counterclockwise(); //turns counterclockwise until the line sensors all match on a line
void robot_drive_til_lines(int lines, int direct);
void check_line_sensors(unsigned long cur_time); //checks the values from line sensors, know if on line and if passing lines
void send_line_sensor_pulse(); //send pulse of light from the line sensors
void robot_passed_line(); //update the robots position since we just passed over a line
void line_on_track(unsigned long cur_time); //robot is following the line successfully
void line_slightly_left(); //robot is slightly to the left of the line
void line_slightly_right(); //robot is slightly to the right of the line
void line_far_left(); //robot is far left of the line
void line_far_right(); //robot is far right of the line
int check_line_sensors_on_line(unsigned long cur_time); //return 1 if all MAIN_SIDE line sensors on line, else return 0
int check_line_sensors_on_line_turn(unsigned long cur_time); //return 1 if all MAIN, MID, SIDE line sensors on line, else return 0
void release_ball(); //stops robot motion and releases the ball
void robot_go_to_location(int final_x, int final_y);
void robot_turn_up(); //turn the robot so its orientation is 0
void normalize_orient(); //sets robot_orient to a value from -pi to pi
void update_line_sensor_vals(unsigned long cur_time);
void check_line_following(unsigned long cur_time); //determine whether line sensor values are on line/passing line/off line. Already need to be updated

//Main Functions
void main_setup(); //main program setup code
void main_loop(); //main program loop code
//Task Functions - Place in setup function
void controlled_locomotion();
void pick_up_game_ball();
void place_game_ball();
void navigate_game_board();
void navigate_hopper();
void move_around_obstacle();
void locate_obstacle();
void gameplay_strategy();


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        MAIN CODE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){
  setup_pins();
  test_fan();
  delay(100000);
  main_setup();
}

void loop(){
  main_loop();
}

void setup_pins(){
  // Setup all of the pins
  if (SERIAL_COMM_BOOL) Serial.begin(SERIAL_BAUD_RATE);
  pinMode(LEFT_WHEEL_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_WHEEL_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(BALL_GRAB_FAN_PIN, OUTPUT);
  //digitalWrite(BALL_GRAB_FAN_PIN, HIGH); //high means the fan is not on
  //pinMode(LEFT_IR_PIN, INPUT); 
  //pinMode(RIGHT_IR_PIN, INPUT);
  fan_servo.attach(BALL_RELEASE_SERVO_PIN);
  delay(100);
  fan_servo.write(BALL_RELEASE_SERVO_INITIAL_VAL);
  delay(100);
  pinMode(BALL_GRAB_FAN_PIN, OUTPUT);
}

void main_setup(){
  int error_check; // Check if any routines don't run as expected. Return 0 if Routine runs error free
  compass_init(); // Get the initial orientation of the robot in the gameboard  
  init_hoppers(); // create structures for the hoppers
  error_check = detect_hoppers(); // Call routine to find position and orientation of hoppers
  if (error_check != 0){ detect_hoppers_error(error_check);} // In event of failure, call backup routine
  error_check = get_first_ball(); // code for getting the first ball from a random hopper
  if (error_check != 0){ get_first_ball_error(error_check);} // In event of failure, call backup routine
}

void main_loop(){
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        ROBOT MOVEMENT
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stop_robot_motion(){
  robot_direction = 0;
  analogWrite(RIGHT_WHEEL_ENABLE_PIN, 0);
  analogWrite(LEFT_WHEEL_ENABLE_PIN, 0);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
}

void start_robot_forward(){
  robot_direction = 1;
  analogWrite(RIGHT_WHEEL_ENABLE_PIN, right_wheel_speed);
  analogWrite(LEFT_WHEEL_ENABLE_PIN, left_wheel_speed);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
}

void start_robot_straight_forward(){
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED;
  start_robot_forward();
}

void start_robot_backward(){
  robot_direction = -1;
  analogWrite(RIGHT_WHEEL_ENABLE_PIN, right_wheel_speed);
  analogWrite(LEFT_WHEEL_ENABLE_PIN, left_wheel_speed);
  digitalWrite(LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
}

void start_robot_straight_backward(){
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED;
  start_robot_backward();
}

void start_robot_clockwise(){
  robot_direction = 2;
  analogWrite(RIGHT_WHEEL_ENABLE_PIN, RIGHT_WHEEL_TURN_SPEED);
  analogWrite(LEFT_WHEEL_ENABLE_PIN, LEFT_WHEEL_TURN_SPEED);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
}

void start_robot_counterclockwise(){
  robot_direction = -2;
  analogWrite(RIGHT_WHEEL_ENABLE_PIN, RIGHT_WHEEL_TURN_SPEED);
  analogWrite(LEFT_WHEEL_ENABLE_PIN, LEFT_WHEEL_TURN_SPEED);
  digitalWrite(LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
}

void robot_quarter_turn_clockwise(){ //the robot turns 90 degrees clockwise, must be starting on a line intersection
  start_robot_clockwise();
  unsigned long cur_time = micros();
  int check = 1;
  while (check != 0){ //need to get off of the initial line
    check = check_line_sensors_on_line_turn(cur_time); 
    cur_time = micros();
  }
  while (check != 1){
    check = check_line_sensors_on_line_turn(cur_time); //keep turning until the line sensors all on a line
    cur_time = micros();
  }
  stop_robot_motion();
  robot_orient += PI_OVER_TWO - abs(robot_orient - ((int) (robot_orient / PI_OVER_TWO) * PI_OVER_TWO));
}

void robot_quarter_turn_counterclockwise(){ //the robot turns 90 degrees clockwise, must be starting on a line intersection
  start_robot_counterclockwise();
  int check = 1;
  unsigned long cur_time = micros();
  while (check != 0){ //need to get off of the initial line, checks for 2 in a row to prevent error
    check = check_line_sensors_on_line_turn(cur_time); 
    cur_time = micros();
  }
  while (check != 1){
    check = check_line_sensors_on_line_turn(cur_time); //keep turning until the line sensors all on a line
    cur_time = micros();
  }
  stop_robot_motion();
  robot_orient -= (robot_orient - abs(((int) (robot_orient / PI_OVER_TWO) * PI_OVER_TWO)));
}

void robot_turn_up(){ //turns robot so that it is facing direction 0
  normalize_orient();
  if (robot_orient >= PI_OVER_TWO + ROBOT_TURN_UP_ERROR) robot_quarter_turn_counterclockwise();
  if (robot_orient <= -1 * PI_OVER_TWO - ROBOT_TURN_UP_ERROR) robot_quarter_turn_clockwise();
  if (robot_orient <= PI_OVER_TWO + ROBOT_TURN_UP_ERROR) robot_quarter_turn_counterclockwise();
  if (robot_orient >= -1 * PI_OVER_TWO - ROBOT_TURN_UP_ERROR) robot_quarter_turn_counterclockwise();
}

void robot_drive_til_line(int line, int direct){ //line is the line number to reach, direct is 0 for x, 1 for y
  while (1){
    unsigned long cur_time = micros();
    check_line_sensors(cur_time);
    if (direct == 0){
      if (x_line_robot == line) break;
    }
    if (direct == 1){
      if (y_line_robot == line) break;
    }
  }
  stop_robot_motion();
}

void robot_go_to_location(int final_x, int final_y){
  normalize_orient(); //forces angle between -pi and pi
  if (x_line_robot < 2){ //need to go up first
    robot_turn_up();
    robot_drive_vertic(2); //drive to second vertical line
  }
  if (abs(robot_orient - PI) < LINE_PASS_ANGLE_ERROR || abs(robot_orient + PI) < LINE_PASS_ANGLE_ERROR){ //go left/right first
    robot_drive_horiz(final_x);
    robot_drive_vertic(final_y);
  }
  else{ //go up/down first
    robot_drive_vertic(final_y);
    robot_drive_horiz(final_x);
  }
  stop_robot_motion();
}

void robot_drive_horiz(int line){ //robot drives to the line specified by line
  normalize_orient();
  while (abs(robot_orient - PI_OVER_TWO) > LINE_PASS_ANGLE_ERROR && abs(robot_orient + PI_OVER_TWO) > LINE_PASS_ANGLE_ERROR){ //not facing left/right
    robot_quarter_turn_clockwise();
  }
  if (line > x_line_robot){ //need to go right
      if(abs(robot_orient - PI_OVER_TWO) < LINE_PASS_ANGLE_ERROR){ //facing to right so drive forward
        start_robot_straight_forward();
      }
      else{ //facing to left so drive backwards
        start_robot_straight_backward();
      }
      robot_drive_til_line(line, 0);  //drive to line
    }
    else { //need to go left
      if(abs(robot_orient + PI_OVER_TWO) < LINE_PASS_ANGLE_ERROR){ //facing to left so drive forward
        start_robot_straight_forward();
      }
      else{ //facing to left so drive backwards
        start_robot_straight_backward();
      }
      robot_drive_til_line(line, 0);  //drive to left line
    }
}

void robot_drive_vertic(int line){ //robot drives to the line specified by line
  if (abs(abs(robot_orient) - PI) > LINE_PASS_ANGLE_ERROR && abs(robot_orient) > LINE_PASS_ANGLE_ERROR){ //not facing up/down
    robot_quarter_turn_clockwise();
  }
  if (line > y_line_robot){ //need to go up
      if(abs(robot_orient) < LINE_PASS_ANGLE_ERROR){ //facing up so drive forward
        start_robot_straight_forward();
      }
      else{ //facing down so drive backwards
        start_robot_straight_backward();
      }
      robot_drive_til_line(line, 1);  //drive to line
    }
    else { //need to go down
      if(abs(abs(robot_orient) - PI) < LINE_PASS_ANGLE_ERROR){ //facing down so drive forward
        start_robot_straight_forward();
      }
      else{ //facing up so drive backwards
        start_robot_straight_backward();
      }
      robot_drive_til_line(line, 1);  //drive to left line
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        LINE FOLLOWING
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void check_line_sensors(unsigned long cur_time){ // CURTIME in MICROSECONDS
  if (cur_time < line_sensor_time) return;
  update_line_sensor_vals(cur_time);
  if (line_sensor_checks == NUM_LINE_SENSORS){
    check_line_following(cur_time);
  }
}

void update_line_sensor_vals(unsigned long cur_time){
  if (line_sensor_state == 0){ //Send out pulse from the line sensors
    send_line_sensor_pulse(); //send pulse
    line_sensor_checks = 0;
    line_sensor_time = cur_time + line_sensor_delay[line_sensor_checks]; //update time to check the sensors
    line_sensor_state = line_sensor_order[line_sensor_checks]; //set state to check for the colours measured
  }
  else { //Check values recorded by the line sensors
    if (line_sensor_state == 1){
      left_line_sensor_val = digitalRead(LEFT_LINE_SENSOR_PIN);
      line_sensor_checks += 1;
      if (line_sensor_checks < NUM_LINE_SENSORS){
        line_sensor_time = cur_time + line_sensor_delay[line_sensor_checks];
        line_sensor_state = line_sensor_order[line_sensor_checks];
      }
      return;
    }
    if (line_sensor_state == 2){
      mid_line_sensor_val = digitalRead(MID_LINE_SENSOR_PIN);
      line_sensor_checks += 1;
      if (line_sensor_checks < NUM_LINE_SENSORS){
        line_sensor_time = cur_time + line_sensor_delay[line_sensor_checks];
        line_sensor_state = line_sensor_order[line_sensor_checks];
      }
      return;
    }
    if (line_sensor_state == 3){
      right_line_sensor_val = digitalRead(RIGHT_LINE_SENSOR_PIN);
      line_sensor_checks += 1;
      if (line_sensor_checks < NUM_LINE_SENSORS){
        line_sensor_time = cur_time + line_sensor_delay[line_sensor_checks];
        line_sensor_state = line_sensor_order[line_sensor_checks];
      }
      return;
    }
    if (line_sensor_state == 4){
      main_line_sensor_val = digitalRead(MAIN_LINE_SENSOR_PIN);
      line_sensor_checks += 1;
      if (line_sensor_checks < NUM_LINE_SENSORS){
        line_sensor_time = cur_time + line_sensor_delay[line_sensor_checks];
        line_sensor_state = line_sensor_order[line_sensor_checks];
      }
      return;
    }
    if (line_sensor_state == 5){
      side_line_sensor_val = digitalRead(SIDE_LINE_SENSOR_PIN);
      line_sensor_checks += 1;
      if (line_sensor_checks < NUM_LINE_SENSORS){
        line_sensor_time = cur_time + line_sensor_delay[line_sensor_checks];
        line_sensor_state = line_sensor_order[line_sensor_checks];
      }
      return;
    }
  }
}

void check_line_following(unsigned long cur_time){
  /*Serial.print(left_line_sensor_val);
  Serial.print('\t');
  Serial.print(mid_line_sensor_val);
  Serial.print('\t');
  Serial.println(right_line_sensor_val);*/
  line_sensor_state = 0; // Ready for next check
  line_sensor_checks = 0;
  line_sensor_time = cur_time + PULSE_LINE_SENSOR_TIME; //add delay before next pulse
  if (side_line_sensor_val == HIGH && main_line_sensor_val == HIGH){ // robot passes a line
    if (robot_on_line != 1){ 
      robot_passed_line(); //passed line
      robot_on_line = 1;
    }
    else{ 
      robot_on_line = 0;
    }
  }
  correction_factor += correction_state;
  if (left_line_sensor_val == LOW && mid_line_sensor_val == HIGH && right_line_sensor_val == LOW && main_line_sensor_val == HIGH){ //on track
    line_on_track(cur_time);
  }
  if (left_line_sensor_val == HIGH && mid_line_sensor_val == HIGH && right_line_sensor_val == LOW){ //slightly too far right
    line_slightly_right();
  }
  if (left_line_sensor_val == HIGH && mid_line_sensor_val == LOW && right_line_sensor_val == LOW){ //too far right
    line_far_right();
  }
  if (right_line_sensor_val == HIGH && mid_line_sensor_val == HIGH && left_line_sensor_val == LOW){ //slightly too far left
    line_slightly_left();
  }
  if (right_line_sensor_val == HIGH && mid_line_sensor_val == LOW && left_line_sensor_val == LOW){ //too far left
    line_far_left();
  }
  if (robot_direction == 1) {
    start_robot_forward();
  }
  else {
    start_robot_backward();
  }
}

void send_line_sensor_pulse(){
  pinMode(LEFT_LINE_SENSOR_PIN, OUTPUT);
  digitalWrite(LEFT_LINE_SENSOR_PIN, HIGH);
  pinMode(MID_LINE_SENSOR_PIN, OUTPUT);
  digitalWrite(MID_LINE_SENSOR_PIN, HIGH); 
  pinMode(RIGHT_LINE_SENSOR_PIN, OUTPUT);
  digitalWrite(RIGHT_LINE_SENSOR_PIN, HIGH);
  pinMode(MAIN_LINE_SENSOR_PIN, OUTPUT);
  digitalWrite(MAIN_LINE_SENSOR_PIN, HIGH);
  pinMode(SIDE_LINE_SENSOR_PIN, OUTPUT);
  digitalWrite(SIDE_LINE_SENSOR_PIN, HIGH);
  delayMicroseconds(10);
  pinMode(LEFT_LINE_SENSOR_PIN, INPUT);
  pinMode(MID_LINE_SENSOR_PIN, INPUT);
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT);
  pinMode(MAIN_LINE_SENSOR_PIN, INPUT);
  pinMode(SIDE_LINE_SENSOR_PIN, INPUT);
}

void robot_passed_line(){
  while(robot_orient > (2*PI)) robot_orient -= (2 * PI); //put the orientation in range 0 to 2PI
  while(robot_orient < 0) robot_orient += (2 * PI);
  if (robot_orient > -1 * LINE_PASS_ANGLE_ERROR && robot_orient < LINE_PASS_ANGLE_ERROR){ //robot is facing upwards
      y_line_robot += robot_direction;
      y_robot = y_line_robot * LINE_SEP_DIST;
  }
  if (robot_orient > PI_OVER_TWO - LINE_PASS_ANGLE_ERROR  && robot_orient < PI_OVER_TWO + LINE_PASS_ANGLE_ERROR){ //robot is facing right
      x_line_robot += robot_direction;
      x_robot = x_line_robot * LINE_SEP_DIST;
  }
  if (robot_orient > PI - LINE_PASS_ANGLE_ERROR && robot_orient < PI + LINE_PASS_ANGLE_ERROR){ //robot is facing down
      y_line_robot -= robot_direction;
      y_robot = y_line_robot * LINE_SEP_DIST;
  }
  if (robot_orient > PI_THREE_OVER_TWO - LINE_PASS_ANGLE_ERROR  && robot_orient < PI_THREE_OVER_TWO + LINE_PASS_ANGLE_ERROR){ //robot is facing left
      x_line_robot -= robot_direction;
      x_robot = x_line_robot * LINE_SEP_DIST;
  }
}

void line_on_track(unsigned long cur_time){
  right_wheel_speed += (int) ((RIGHT_WHEEL_MAX_SPEED - right_wheel_speed) * (1 - 1/ (abs(correction_factor) + 1))); //bring both wheels closer to max speeds
  left_wheel_speed += (int) ((LEFT_WHEEL_MAX_SPEED - left_wheel_speed) * (1 - 1/ (abs(correction_factor) + 1)));
  if (off_track_flag == 1){
    off_track_flag = 0;
    on_track_time = cur_time + GOOD_ON_TRACK_TIME;
    return;
  }
  if (cur_time > on_track_time){ //have been on line for good amount of time
    correction_state = 0;
    correction_factor = 0;
    right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
    left_wheel_speed = LEFT_WHEEL_MAX_SPEED;
  }
}

void line_slightly_left(){
  off_track_flag = 1;
  if (correction_factor == 0) correction_factor += correction_state; //prevent div by 0
  if (correction_state == 2){ //coming from being far left
    left_wheel_speed = (int) (LEFT_WHEEL_MAX_SPEED - (RIGHT_WHEEL_MAX_SPEED - right_wheel_speed) * (1 - 1/ (abs(correction_factor) + 1))); //turn opposite direction
    right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
    correction_state = -1; //now turning left
  }
  if (correction_state == -1){ //correcting slightly turning robot
    if (correction_factor > 0){ //robot was far to the left
      left_wheel_speed += (int) ((LEFT_WHEEL_MAX_SPEED - left_wheel_speed) * (1.0 - 1/ (abs(correction_factor) + 1)));
    }
    if (correction_factor < 0){ //robot was slightly right
      right_wheel_speed = (int) (RIGHT_WHEEL_MAX_SPEED - (LEFT_WHEEL_MAX_SPEED - left_wheel_speed) * (1 - 1/ (abs(correction_factor) + 1))); //turn opposite direction slightly
      left_wheel_speed = LEFT_WHEEL_MAX_SPEED;
    }
  }
  if (correction_state == 0){
    correction_state = 1;
    left_wheel_speed = LEFT_WHEEL_MAX_SPEED - MIN_SPEED_CORRECTION_VALUE;
    right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
  }
}

void line_slightly_right(){
  off_track_flag = 1;
  if (correction_factor == 0) correction_factor += correction_state; //prevent div by 0
  if (correction_state == -2){ //coming from being far right
    right_wheel_speed = (int) (RIGHT_WHEEL_MAX_SPEED - (LEFT_WHEEL_MAX_SPEED - left_wheel_speed) * (1.0 - 1/ (abs(correction_factor) + 1))); //turn opposite direction
    left_wheel_speed = LEFT_WHEEL_MAX_SPEED;
    correction_state = 1; //now turning right
  }
  if (correction_state == 1){ //correcting slightly turning robot
    if (correction_factor < 0){ //robot was far to the right
      right_wheel_speed += (int) ((RIGHT_WHEEL_MAX_SPEED - right_wheel_speed) * (1.0 - 1/ (abs(correction_factor) + 1)));
    }
    if (correction_factor > 0){ //robot was slightly left
      left_wheel_speed = (int) (LEFT_WHEEL_MAX_SPEED - (RIGHT_WHEEL_MAX_SPEED - right_wheel_speed) * (1.0 - 1/ (abs(correction_factor) + 1))); //turn opposite direction slightly
      right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
    }
  }
  if (correction_state == 0){
    correction_state = -1;
    left_wheel_speed = LEFT_WHEEL_MAX_SPEED - MIN_SPEED_CORRECTION_VALUE;
    right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
  }
}

void line_far_left(){ //too far left
  off_track_flag = 1;
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED - MAX_SPEED_CORRECTION_VALUE;
  correction_state = 2;
  if (correction_factor < 0) correction_factor = 1; //robot thinks its angled right, so reset this so it will think its angled left
}

void line_far_right(){
  off_track_flag = 1;
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED - MAX_SPEED_CORRECTION_VALUE;
  correction_state = -2;
  if (correction_factor > 0) correction_factor = -1; //robot thinks its angled left, so reset this so it will think its angled right
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        POSITION TRACKING+ENCODER
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void update_location(unsigned long cur_time){
  if (cur_time > wheel_ir_time){
    int left_ir = analogRead(LEFT_IR_PIN);
    if (left_ir < 5) left_ir = 0;
    if (left_ir >= 5) left_ir = 1;
    //int right_ir = digitalRead(RIGHT_IR_PIN); //not used
    if (robot_direction == 1 || robot_direction == -1){ //robot is driving straight forward or backwards
      if (prev_ir_left == 0 && left_ir == 1){
        //x_robot += cos(robot_orient) * robot_direction * WHEEL_CIRCUMFERENCE / WHEEL_NUM_HOLES; // TEMP
        Serial.println(x_robot);
        x_robot += sin(robot_orient) * robot_direction * WHEEL_CIRCUMFERENCE / WHEEL_NUM_HOLES;
        y_robot += cos(robot_orient) * robot_direction * WHEEL_CIRCUMFERENCE / WHEEL_NUM_HOLES;
      }
    }
    if (robot_direction == 2){ //robot is turning clockwise
      if (prev_ir_left == 0 && left_ir == 1){
        robot_orient += (WHEEL_CIRCUMFERENCE / WHEEL_NUM_HOLES) / ROBOT_TURNING_RADIUS;
      }
    }
    if (robot_direction == -2){ //robot is turning counterclockwise
      if (prev_ir_left == 0 && left_ir == 1){
        robot_orient -= (WHEEL_CIRCUMFERENCE / WHEEL_NUM_HOLES) / ROBOT_TURNING_RADIUS;
      }
    }
    wheel_ir_time = cur_time + WHEEL_IR_DELAY;
    prev_ir_left = left_ir;
    //prev_ir_right == right_ir;
  }
}

int check_line_sensors_on_line(unsigned long cur_time){ //return 1 if all MAIN, SIDE line sensors on line, else return 0
  if (cur_time >= line_sensor_time){
    update_line_sensor_vals(cur_time);
    if (main_line_sensor_val == HIGH && side_line_sensor_val == HIGH) return 1;
    return 0;
  }
  else{
    return 2; //didnt check
  }
}

int check_line_sensors_on_line_turn(unsigned long cur_time){ //return 1 if all MAIN, MID, SIDE line sensors on line, else return 0
  if (cur_time >= line_sensor_time){
    update_line_sensor_vals(cur_time);
    if (main_line_sensor_val == HIGH && side_line_sensor_val == HIGH && mid_line_sensor_val) return 1;
    return 0;
  }
  else{ 
    return 2; //didnt check since not at right time
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        HOPPER DETECTION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int detect_hoppers(){
  hopper_detect_initial_pos(); // get to initial position for detecting hoppers
  unsigned long cur_time = micros(); //Record the current robot time
  ping_time = cur_time; // initialize ping time
  if (wheel_ir_time == 0){
    wheel_ir_time = cur_time;}
  int prev_dist = 0; // Saves last measured important distance
  int cur_hopper = 0; //record current hopper we are finding, either 0 or 1
  int flag = 1; // 1 if we are ready to detect a new hopper, 2 if we found 3rd leg, else 0
  int ping_dist = 0; // keeps track of the distance measured by the most recent ping
  int prev_dist2 = 0; //keeps track of previous ping
  int count = 0; // keeps track of the number of similar distance pings in a row
  while (1){ //Run until end is reached
    cur_time = micros(); //update current time every loop
    update_location(cur_time); // updates x_robot and y_robot
    if (cur_time > ping_time){ // send another ping if enough time has passed
      unsigned int ping_record_time = hopper_detector.ping(); //measures time to receive ping
      ping_dist = ping_record_time / US_ROUNDTRIP_CM + (int) y_robot; // ping distance from gamefield bottom
      //Serial communcation
      if (SERIAL_COMM_BOOL) serial_comm(cur_time, prev_dist, prev_dist2, cur_hopper, flag, ping_dist, count);
      ping_time += HOP_DETECT_PING_DELAY; // add delay before another ping is sent
      if (ping_dist > HOP_DETECT_MAX_DIST) continue; //prevent sonar malfunctions from affecting anything
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
    if (ping_dist > HOP_DETECT_MIN_DIST + (int) y_robot && flag == 1 && count >= HOPPER_VALUES_IN_ROW){ // found a hopper
      if (prev_dist == 0){ // Found first leg of hopper
        prev_dist = ping_dist; // prev dist stores first leg of hopper distance
        prev_dist2 = ping_dist;
        count = 0;
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
  //display_hoppers(); //Shows data for each hopper on serial
  return 0;
}

void display_hoppers(){
  stop_robot_motion(); //turn off wheel motors
  if (SERIAL_COMM_BOOL || 1){
    for (int x = 0; x < TEST_SEVEN_NUM_HOPPERS; x++){
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


void detect_hoppers_error(int error){
  // Set error flags so that robot assumes just one random hopper in middle, but don't drive through middle error
}

void hopper_detect_initial_pos(){
  // drive the robot to a position on left side of board just outside of the starting area
  // update current location
  /*x_robot = 0.0;
  y_robot = 0.0;
  //drive forward
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);*/
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        SERIAL COMMUNICATION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void serial_comm(unsigned long cur_time, int prev_dist, int prev_dist2, int cur_hopper, int flag, int ping_dist, int count){
  if (SERIAL_COMM_BOOL){
    Serial.print(ping_time);
    Serial.print(',');
    Serial.print(wheel_ir_time);
    Serial.print(',');
    Serial.print(prev_ir_left);
    Serial.print(',');
    Serial.print(prev_ir_right);
    Serial.print(',');
    Serial.print(robot_direction);
    Serial.print(',');
    Serial.print(initial_orient_robot);
    Serial.print(',');
    Serial.print(x_robot);
    Serial.print(',');
    Serial.print(y_robot);
    Serial.print(',');
    Serial.print(robot_orient);
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        PICK UP GAMEBALL
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pick_up_game_ball(){
  fan_servo.write(BALL_RELEASE_SERVO_INITIAL_VAL);
  digitalWrite(BALL_GRAB_FAN_PIN, LOW); //low means the fan is sucking up the ball
  delay(BALL_SUCTION_ON_TIME);
  digitalWrite(BALL_GRAB_FAN_PIN, HIGH);
  delay(BALL_SUCTION_OFF_TIME);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        PLACE GAMEBALL
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void place_game_ball(){
  left_wheel_speed = LEFT_WHEEL_GAMEBOARD_SPEED;
  right_wheel_speed = RIGHT_WHEEL_GAMEBOARD_SPEED;
  start_robot_forward();
  //if (abs(TEST_THREE_INITIAL_ORIENT - PI_OVER_TWO) < LINE_PASS_ANGLE_ERROR) start_robot_forward();
  //if(abs(TEST_THREE_INITIAL_ORIENT - PI_THREE_OVER_TWO) < LINE_PASS_ANGLE_ERROR) start_robot_backward();
  while(abs(x_robot - column_locations[TEST_THREE_FINAL_COLUMN]) > BALL_RELEASE_POSITION_ERROR){
    unsigned long cur_time = micros();
    update_location(cur_time);
  }
  stop_robot_motion();
  release_ball();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        OTHER FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void normalize_orient(){
  if (robot_orient < 0){
    while (robot_orient < -1 * PI) robot_orient += 2 * PI;
  }
  else{
    while (robot_orient > PI) robot_orient -= 2 * PI;
  }
}

void compass_init(){
  // save the current compass orientation to global variables
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

void controlled_locomotion(){
  x_line_robot = TEST_ONE_INITIAL_X;
  y_line_robot = TEST_ONE_INITIAL_Y;
  x_robot = TEST_ONE_INITIAL_X * LINE_SEP_DIST;
  y_robot = TEST_ONE_INITIAL_Y * LINE_SEP_DIST;
  robot_orient = TEST_ONE_INITIAL_ORIENT;
  robot_go_to_location(TEST_ONE_FINAL_X, TEST_ONE_FINAL_Y);
}

void navigate_game_board(){
  x_line_robot = TEST_FOUR_INITIAL_X;
  y_line_robot = TEST_FOUR_INITIAL_Y;
  x_robot = TEST_FOUR_INITIAL_X * LINE_SEP_DIST;
  y_robot = TEST_FOUR_INITIAL_Y * LINE_SEP_DIST;
  robot_orient = TEST_FOUR_INITIAL_ORIENT;
  if (x_robot < GAME_FIELD_WIDTH){ //go to left position
    robot_go_to_location(GAME_BOARD_LEFT_X, GAME_BOARD_LEFT_Y);
  }
  else{ //go to right position
    robot_go_to_location(GAME_BOARD_RIGHT_X, GAME_BOARD_RIGHT_Y);
  }
  robot_turn_up();
  left_wheel_speed = LEFT_WHEEL_GAMEBOARD_SPEED;
  right_wheel_speed = RIGHT_WHEEL_GAMEBOARD_SPEED;
  start_robot_forward();
  while (y_robot < RELEASE_BALL_Y_VALUE){//drive until right y distance
    int cur_time = micros();
    update_location(cur_time);
  }
  stop_robot_motion();
  robot_quarter_turn_counterclockwise(); //align robot so it is parallel to gameboard
  start_robot_forward();
  while (abs(x_robot - RELEASE_BALL_X_VALUE) > BALL_RELEASE_POSITION_ERROR){//drive until right x distance
    int cur_time = micros();
    update_location(cur_time);
  }
  stop_robot_motion();
}

void navigate_hopper(){
  
}
void move_around_obstacle(){
  
}

void locate_obstacle(){ //start facing to the right, y = 2
  init_hoppers();
  robot_drive_horiz(7);
  y_line_robot = TEST_SEVEN_INITIAL_Y;
  x_line_robot = TEST_SEVEN_INITIAL_X;
  y_robot = TEST_SEVEN_INITIAL_Y * LINE_SEP_DIST;
  void start_robot_straight_forward(); //turn on wheel forwards to move forward
  unsigned long cur_time = micros(); //Record the current robot time
  ping_time = cur_time; // initialize ping time
  int prev_dist = 0; // Saves last measured important distance
  int cur_hopper = 0; //record current hopper we are finding, either 0 or 1
  int flag = 1; // 1 if we are ready to detect a new hopper, 2 if we found 3rd leg, else 0
  int ping_dist = 0; // keeps track of the distance measured by the most recent ping
  int prev_dist2 = 0; //keeps track of previous ping
  int count = 0; // keeps track of the number of similar distance pings in a row
  while (1){ //Run until end is reached
    cur_time = micros(); //update current time every loop
    //check_line_sensors(cur_time); //follow line
    if (cur_time > ping_time){ // send another ping if enough time has passed
      unsigned int ping_record_time = hopper_detector.ping(); //measures time to receive ping
      update_location(cur_time);
      ping_dist = ping_record_time / US_ROUNDTRIP_CM + (int) y_robot; // ping distance from gamefield bottom
      ping_time += HOP_DETECT_PING_DELAY; // add delay before another ping is sent
      if (ping_dist > HOP_DETECT_MAX_DIST) continue; //prevent sonar malfunctions from affecting anything
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
        if (ping_dist > prev_dist + HOPPER_DETECT_ERROR && count == HOPPER_VALUES_IN_ROW) flag = 2; // found the 3rd leg
      }
    }
    //if (flag == 2 && ping_dist - y_robot < HOPPER_DETECT_Y_ERROR && count == HOPPER_VALUES_IN_ROW){ // Past the hopper
      // WONT WORK SINCE DOESNT RETURN 0, SO NEW IF STATEMENT FIXES IT
    if (flag == 2 && abs(prev_dist - ping_dist) > HOPPER_DETECT_ERROR && count == HOPPER_VALUES_IN_ROW){ // Past the hopper
      prev_dist = 0; //set prev_dist back to 0
      flag = 1; //ready to search for next hopper
      count = 0; //reset count to 0 since we have past the hopper
    }
    if (ping_dist > HOP_DETECT_MIN_DIST + (int) y_robot && flag == 1 && count == HOPPER_VALUES_IN_ROW){ // found a hopper
    //if (ping_dist > HOP_DETECT_MIN_DIST + (int) y_robot){ // found a hopper
      if (prev_dist == 0){ // Found first leg of hopper
        prev_dist = ping_dist; // prev dist stores first leg of hopper distance
        prev_dist2 = ping_dist;
        count = 0;
        continue;
      }
      //found middle of hopper
      if (ping_dist > prev_dist + HOPPER_DETECT_ERROR && ping_dist < prev_dist + HOPPER_DETECT_MAX_ERROR){ // Hopper is Orientation 0
      //if (ping_dist > prev_dist + HOPPER_DETECT_ERROR){ // Hopper is Orientation 0
        hoppers[cur_hopper].x = (int) x_robot / LINE_SEP_DIST;
        hoppers[cur_hopper].y = prev_dist / LINE_SEP_DIST;
        hoppers[cur_hopper].orient = 0;
        hoppers[cur_hopper].balls = RANDOM_HOPPER_BALLS_NUM;
        cur_hopper++; //start looking for the next hopper
        flag = 0;
        prev_dist = ping_dist; //update prev_dist to distance to the middle leg
        count = 0; //found middle leg of hopper so reset count to 0
      }
      if (ping_dist < prev_dist - HOPPER_DETECT_ERROR && ping_dist > prev_dist - HOPPER_DETECT_MAX_ERROR){ // Hopper is Orientation 1
      //if (ping_dist < prev_dist - HOPPER_DETECT_ERROR){ // Hopper is Orientation 1
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
    if (cur_hopper >= TEST_SEVEN_NUM_HOPPERS) break; //only detect 1 hopper for this test
    prev_dist2 = ping_dist; //update new previous distance value
    // If the robot reaches the end of the path (past sensors) return error
    }
  display_hoppers(); //Shows data for each hopper on serial
  return;
}
void gameplay_strategy(){
  
}

void release_ball(){
  //stop_robot_motion();
  fan_servo.write(BALL_RELEASE_SERVO_FINAL_VAL);
  delay(RELEASE_BALL_RESET_DELAY);
  fan_servo.write(BALL_RELEASE_SERVO_INITIAL_VAL);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        TEST
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void test_fan(){
  pick_up_game_ball();
  release_ball();
  /*fan_servo.write(BALL_RELEASE_SERVO_FINAL_VAL);
  delay(2000);
  fan_servo.write(BALL_RELEASE_SERVO_INITIAL_VAL);*/
  //fan_servo.write(BALL_RELEASE_SERVO_FINAL_VAL);
}
