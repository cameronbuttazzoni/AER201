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
            1.9:
                - Changed line following algorithm depending on whether driving forward or backwards
            1.10:
                - Finished movement code, finished main code running functions, still need to add orient_on_hopper code, debug and cleanup
            1.11:
                - Robot motion around gameboard always drives forward now
            1.12:
                - Added aligning to hopper function, updated all defined values, added code to scan gameboard and 
                      

Need to Change List:

Potential Problems List:
    - Only uses one wheel's IR sensor, so assumes both wheels spin at a constant speed

Need to Add List:
    - int grab_ball()
    - end location for Hopper Detection
    - As possible errors are found in function, update their corresponding error handler
    - Add emergency stop interrupt

*/
#include <NewPing.h>
#include <Servo.h>

//#include <math.h>

//Defined Constants

//INIT CONSTANTS
#define ROBOT_INITIAL_X 4 //iniital x line the robot starts on
#define ROBOT_INITIAL_Y 1 //initial y line the robot starts on
#define ROBOT_INITIAL_ORIENT PI_OVER_TWO //initial robot orientation

// HOPPER DETECTION
#define SERIAL_COMM_BOOL 1 //1 if using serial communication, else 0
#define HOP_DETECT_TRIG_PIN 53 //Hopper detecting distance sensor's trigger pin
#define HOP_DETECT_ECHO_PIN 52 //Hopper detecting distance sensor's echo pin
#define HOP_DETECT_MAX_DIST 50 //Maximum distance values collected (in cm)
#define HOP_DETECT_MIN_DIST 5 //Minimum distance values polled (in cm), keep > 0
#define HOP_DETECT_PING_DELAY 30000 //time between pings from hopper detector distance sensor (min 30000) microseconds
#define HOPPER_DETECT_ERROR 3 // distance differences where program thinks it has detected change
#define HOPPER_DETECT_MAX_ERROR 25 // max difference between leg values that trigger difference
#define HOPPER_DETECT_Y_ERROR 2 // distance that counts as the same value when comparing to y_robot
#define HOPPER_VALUES_IN_ROW 2 //Number of similar values in a row that need to be found
#define HOPPER_DETECT_INITIAL_X 40 //go to this x value (cm) for the initial hopper detection algorithm
#define HOP_DETECT_X_OFFSET 5 //distance from the distance sensor to the centre of the robot in gamefield x-direction (orientation is -PI/2)
#define HOP_DETECT_Y_OFFSET 10 //distance from the distance sensor to the centre of the robot in gamefield y-direction (orientation is -PI/2)
#define HOP_DETECT_X_ERROR 1 //error in the x position of hopper, this is added to its estimated x-location, put low positive eg 1

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
#define GAMEFIELD_RED_LINE_NUM 4 //the value of the center line in terms of which x-line it is

// WHEEL CONSTANTS
#define LEFT_IR_PIN 0 //analog pin for the left wheel's IR sensor
#define RIGHT_IR_PIN A0 //analog pin for the right wheel's IR sensor
#define WHEEL_IR_DELAY 5000 //time between checks for the wheel IR sensors MICROSECONDS
#define WHEEL_CIRCUMFERENCE 28.3 //circumference of the wheel (in cm)
#define WHEEL_NUM_HOLES 18 //number of holes in the wheel
#define POSITION_ERROR 2 //in cm must be within this distance for position checks MAKE SMALLER THEN MIN ENCODER CHANGE
#define IR_DIFFERENCE_VAL 10 //greater than or equal to this val counts as the sensors detecting, less than this is not detecting

// ROBOT DIMENSIONS
#define ROBOT_WIDTH 10 //half the robot width in cm
#define ROBOT_MAX_WIDTH 15 //half the maximum robot width in cm
#define ROBOT_LENGTH_FRONT 10//distance from point of rotation to front of robot
#define ROBOT_LENGTH_BACK 5 //distance from point of rotation to back of robot
#define ROBOT_TURNING_RADIUS 5 //distance from the center of rotation to the wheels of the robot
#define ROBOT_BALL_DROP_X_DIST 5 //the number of cm the ball is from the wheel axis in direction perpendicular to wheel axis
#define ROBOT_BALL_DROP_Y_DIST 5 //the number of cm the ball is from the wheel axis in direction parallel to wheel axis

// MOTOR CONSTANTS
#define RIGHT_WHEEL_ENABLE_PIN 2 //enable the right wheel motor
#define LEFT_WHEEL_ENABLE_PIN 3 //enable the left wheel motor
#define LEFT_FORWARD_PIN 24 //Set to high to move left wheel forward
#define LEFT_BACKWARD_PIN 22 //Set to high to move left wheel backward
#define RIGHT_FORWARD_PIN 26 //set to high to move right wheel forward
#define RIGHT_BACKWARD_PIN 28 //set to high to move right wheel backward
#define LEFT_WHEEL_MAX_SPEED 255 //max of 255
#define RIGHT_WHEEL_MAX_SPEED 255 //max of 255
#define LEFT_WHEEL_TURN_SPEED 130 //max of 255, set to 90
#define RIGHT_WHEEL_TURN_SPEED 130 //max of 255, set to 90
#define LEFT_WHEEL_GAMEBOARD_SPEED 155 //speed of left wheel when moving along gameboard
#define RIGHT_WHEEL_GAMEBOARD_SPEED 155 //speed of right wheel when moving along gameboard
#define LEFT_WHEEL_ALIGN_SPEED 85 //speed of left wheel when aligning to the line, set to 70
#define RIGHT_WHEEL_ALIGN_SPEED 85 //speed of left wheel when aligning to the line, set to70
#define LEFT_WHEEL_ENCODER_SPEED 120 //speed of the left wheel when using the encoders during orienting on a hopper stage
#define RIGHT_WHEEL_ENCODER_SPEED 120 //speed of the right wheel when using the encoders during orienting on a hopper stage
#define LEFT_WHEEL_ORIENT_SPEED 90 //speed of the left wheel when aligning to a hopper
#define RIGHT_WHEEL_ORIENT_SPEED 990 //speed of the right wheel when aligning to a hopper
#define TURN_CORRECTION_DELAY 350 //after quarter turn reverse turn for this long to correct for overturning

// LINE SENSORS
#define NUM_LINE_SENSORS 5 //number of line sensors
#define LEFT_LINE_SENSOR_PIN 45 //left line detecting black/white IR sensor pin
#define MID_LINE_SENSOR_PIN 47 //middle line detecting black/white IR sensor pin
#define RIGHT_LINE_SENSOR_PIN 49 //right line detecting black/white IR sensor pin
#define MAIN_LINE_SENSOR_PIN 51 //the line sensor keeping track of in between the wheels, at very center of robot
#define SIDE_LINE_SENSOR_PIN 43 //line sensor on RIGHT/LEFT side of robot
#define CHECK_LINE_SENSOR_TIME 450 //MICROSECONDS since the line sensors start to check for black/white use 750
#define PULSE_LINE_SENSOR_TIME 50000 //send a pulse every CHECK_LINE_SENSOR_TIME + PULSE_LINE_SENSOR_TIME MICROSECONDS
#define LINE_SENSOR_DELAY 10 //delay in milliseconds between checks to the line sensors NOT USED
#define LINE_PASS_ANGLE_ERROR 0.75//angle in radians must be within for passing lines to register position updates
#define LINE_POSITION_ERROR 5 //error in position when going over line in cm
#define GOOD_ON_TRACK_TIME 2000000 //microseconds that the robot should be on the line for
#define MIN_SPEED_CORRECTION_VALUE 15 //minimum value to change wheel speed by when veering off course
#define MAX_SPEED_CORRECTION_VALUE 40 //max value to change wheel speed for slight off course
#define ROBOT_TURN_UP_ERROR 0.05 //radians, make small
#define ROBOT_ON_LINE_ERROR 3 //number of checks after line before we start checking if we passed line again
#define ROBOT_ON_LINE_TURN_ERROR 8 //number of checks of 0 before we check for line

// BALL RELEASE AND PICKUP CONSTANTS
#define BALL_GRAB_FAN_PIN 38 //pin that controls the fan to suck up balls
#define BALL_GRAB_FAN_CONST_PIN 40 //always keep this negative
#define BALL_RELEASE_SERVO_PIN 50 //pin that controls the servo motor to release the ball
#define BALL_SUCTION_ON_TIME 7000 //amount of time the vacuum sucks for when picking up a ball MILLISECONDS
#define BALL_SUCTION_OFF_TIME 5000 //amount of time the robot delays for after turning off the fan MILLISECONDS
#define BALL_RELEASE_SERVO_INITIAL_VAL 136 //value for when the servo is closed
#define BALL_RELEASE_SERVO_FINAL_VAL 102 //value for when the servo is open
#define BALL_RELEASE_POSITION_ERROR 2 //in cm if the robot is within this distance of the column it will stop to place a ball
#define RELEASE_BALL_RESET_DELAY 2000 //delay time to reset the servo after placing a ball in MILLISECONDS
#define RELEASE_BALL_Y_VALUE 165 //y_robot for when the robot is aligned to the game board
#define RELEASE_BALL_X_VALUE 40 //should be greater than 40
#define GAMEBOARD_INITIAL_X_ROBOT 6 //x line that the robot goes to when going to the gameboard
#define GAMEBOARD_INITIAL_Y_ROBOT 8 //y line that the robot goes to when going to the gameboard
#define FIRST_HOPPER_NUM 3 // the very first ball is from this hopper
#define BALL_GRAB_TRIG_PIN 35 //trig pin for ball grabbing ultrasonic sensor
#define BALL_GRAB_ECHO_PIN 37 //echo pin for ball grabbing ultrasonic sensor
#define BALL_GRAB_MAX_DIST 50 //max dist
#define BALL_GRAB_PING_DELAY 30000 //delay
#define NUMBER_OF_HOP_ORIENT_DISTS 2 //must be even number, half this value is the number of values in a row that are averaged
#define BALL_GRAB_PROPER_DISTANCE 0 //distance the robot must be from the hopper to grab a ball
#define BALL_GRAB_DISTANCE_ERROR 0.5 //distance the robot must be within for the robot to try and grab the ball
#define ZERO_MEASUREMENTS_IN_ROW 2

//Scan gameboard
#define ZERO_POSITION_IR_PIN A1 //the row 0 ir pin
#define ONE_POSITION_IR_PIN A2 //the row 0 ir pin
#define TWO_POSITION_IR_PIN A3 //the row 0 ir pin
#define THREE_POSITION_IR_PIN A4 //the row 0 ir pin
#define FOUR_POSITION_IR_PIN A5 //the row 0 ir pin
#define FIVE_POSITION_IR_PIN A6 //the row 0 ir pin
#define END_OF_SCAN_X 40 //stop scanning gameboard when this x is reached

// OTHER CONSTANTS
#define ROBOT_FOLLOW_LEFT_COLUMN_VAL 1 //go up and down this line when on left side of board
#define ROBOT_FOLLOW_RIGHT_COLUMN_VAL 7 //go up and down this line when on right side of board
#define MID_ZONE_TOP_Y 6 // the y line of the top of the middle zone
#define MID_ZONE_BOT_Y 3 // the y line of the bot of the middle zone
#define SERIAL_BAUD_RATE 9600 //Serial bits per second
#define STOP_MOTION_DELAY 300 //verytime you stop motion, delay for this amount of time

// MATH
#define PI_OVER_TWO 1.5708
#define PI_THREE_OVER_TWO 4.7124
#define EPSILON 0.01

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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        GLOBAL VARIABLES
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int column_locations[] = {65, 70, 75, 80, 85, 90, 95}; //stores the x-coordinates of each of the gameboard columns WRONG
const int line_sensor_order[] = {5, 2, 3, 1, 4}; //1 is left sensor, 2 is middle sensor, 3 is right sensor, 4 is main sensor, 5 is side sensor
const int line_sensor_delay[] = {220, 0, 0, 0, 0}; //cumulative delay, must be in order from smallest to largest 
const int ldr_wall_vals[] = {10, 10, 10, 10, 10, 10};
const int ldr_ball_vals[] = {10, 10, 10, 10, 10, 10};
int ldr_vals[] = {0};
int line_sensor_checks = 0;
int left_line_sensor_val = HIGH; //last measured value of each line sensor
int mid_line_sensor_val = HIGH;
int right_line_sensor_val = HIGH;
int main_line_sensor_val = HIGH;
int side_line_sensor_val = HIGH;
NewPing hopper_detector(HOP_DETECT_TRIG_PIN, HOP_DETECT_ECHO_PIN, HOP_DETECT_MAX_DIST); // Create NewPing object for hopper detection
NewPing ball_grab_detector(BALL_GRAB_TRIG_PIN, BALL_GRAB_ECHO_PIN, BALL_GRAB_MAX_DIST); // Create NewPing object for hopper detection
unsigned long ping_time = 0;     // holds time of the next ping
int ping_dist = 0;
unsigned long front_ping_time = 0;
float front_ping_dist = 0;
unsigned long wheel_ir_time = 0;     // holds time of the next check of wheel IR sensors
unsigned long line_sensor_time = 0; //holds time of the next check of the line sensors
int line_sensor_state = 0; //if 0 need to turn on line sensor reflect, if 1 need to check for colour
int prev_ir_left = 0; //stores the last value measured by the left wheel IR
int prev_ir_right = 0; //stores the last value measured by the right wheel IR
int robot_direction; //1 if moving forward, -1 if moving backward, 2 if turning clockwise, -2 counterclockwise
float x_robot; // holds robot's current x-axis location
float y_robot; // holds robot's current y-axis location
int x_line_robot; //the number of the line horizontally that the robot has crossed, left is 0
int y_line_robot; //the number of the line vertically that the robot has crossed, bottom is 0
float robot_orient = 0; //holds robot's orientation, 0 is towards gameboard, clockwise is positive in radian
Hopper hoppers[NUMBER_OF_HOPPERS]; // Saves the four game field hopper structures
int hopper_order[4] = {0, 1, 2, 3}; //best order to get the hoppers in
int gameboard[NUMBER_BOARD_COLUMNS][NUMBER_BOARD_ROWS] = {0}; // 0 = no ball, 1 = our ball, 2 = their ball, 3 = probably our ball, 4 = probably their ball
unsigned int next_ball_column = 0; //holds the column to play the next ball into?
unsigned int next_hopper; //holds the next hopper to get a ball from
int left_wheel_speed = LEFT_WHEEL_MAX_SPEED; //speed of left wheel
int right_wheel_speed = RIGHT_WHEEL_MAX_SPEED; //speed of right wheel
int correction_state = 0; //0 = no correction, 1 is turning right, 2 turning hard right, -1 is turning left, -2 is turning hard left
int correction_factor = 0; //Indicates the degree of line correction the robot has experienced
int off_track_flag = 0; //has value of 1 if last line check was off the line
unsigned long on_track_time = 0; //records the last time the robot was on track
Servo fan_servo;
int robot_on_line = ROBOT_ON_LINE_ERROR; //keeps track if the robot was on line from the last poll, >0 if was on line, else 0

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup_pins();
void setup_consts();
int detect_hoppers(); // detect hopper routine. Goes to initial position to left of hoppers then moves right and scans
void detect_hoppers_error(int error);
void update_location(unsigned long cur_time);
void hopper_detect_initial_pos(); // go to initial position to the left of the hoppers
void init_hoppers(); // set up hoppers array
int get_first_ball(); // move the robot in position to grab the first ball
void get_first_ball_error(int error);
int pre_board_scan(); // scan the board and update all new balls as opponent played balls
void pre_board_scan_error(int error);
int game_strategy(); // based on the current game board state, updates next_ball_column to the column we will play ball into
void game_strategy_error(int error);
int post_board_scan(); //scan the board and update 1 new ball as ours, new balls > 1, there is uncertainty in which is ours
void post_board_scan_error(int error);
int orient_on_hopper(); //orient on the next hopper
void orient_on_hopper_error(int error); //if orientation was unsuccessful
//handles all serial communication NOTE: print newline character after function call
void serial_comm(unsigned long cur_time, int prev_dist, int prev_dist2, int cur_hopper, int flag, int ping_dist, int count); 
void stop_robot_motion(); //turn off wheel motors
void start_robot_forward(); //turn on wheels to move forward
void start_robot_straight_forward(); //drive full speed straight forward
void start_robot_backward(); //turn on wheels to move backwards
void start_robot_straight_backward(); //drive full speed straight backwards
void start_robot_clockwise(); //turn on wheels to turn clockwise
void start_robot_counterclockwise(); //turn on wheels to turn counterclockwise
void start_robot_clockwise_align(); //turn on wheels to turn clockwise for alignment (slower)
void start_robot_counterclockwise_align(); //turn on wheels to turn counterclockwise for alignment (slower)
void start_robot_clockwise_orient(); //turn on wheels to turn clockwise for orienting on hopper
void start_robot_counterclockwise_orient(); //turn on wheels to turn counterclockwise for orienting on hopper
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
int check_line_sensors_on_line(unsigned long cur_time); //return 1 if SIDE line sensor on line, else return 0
int check_line_sensors_on_line_turn(unsigned long cur_time); //return 1 if all MAIN, MID, SIDE line sensors on line, else return 0
void release_ball(); //stops robot motion and releases the ball
void robot_go_to_location(int final_x, int final_y);
void robot_go_to_gameboard();
void robot_turn_up(); //turn the robot so its orientation is 0
void robot_turn_right(); // turn the robot so its orientation is PI/2
void robot_turn_right(); //turn the robot so its orientation is -PI/2
void robot_turn_down(); //turn the robot so its orientation is PI
void normalize_orient(); //sets robot_orient to a value from -pi to pi
void update_line_sensor_vals(unsigned long cur_time);
void check_line_following(unsigned long cur_time); //determine whether line sensor values are on line/passing line/off line. Already need to be updated
void robot_is_lost(); //while line following, main, left, mid, right sensors all get off line!!!
void align_robot(); //when robot is on line, align it to line
void update_line_sensors(); // updates the values of all of the line sensors
void send_hopper_detect_ping(unsigned long cur_time); //send ping and update ping_dist variable
void send_hopper_orient_ping(unsigned long cur_time);
int go_back_to_line(); //after getting a gameball, go back to the line
void go_back_to_line_error(int error_check); 
void orient_to_hopper(); //get right angle for grabbing ball from hopper
void align_to_hopper(); //get right distance for grabbing ball from hopper

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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        MAIN CODE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){
  setup_pins();
  stop_robot_motion();
  delay(3000);
  detect_hoppers();
  delay(500000);
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
  pinMode(BALL_GRAB_FAN_CONST_PIN, OUTPUT);
  digitalWrite(BALL_GRAB_FAN_CONST_PIN, HIGH);
  digitalWrite(BALL_GRAB_FAN_PIN, HIGH); //high means the fan is not on
  pinMode(LEFT_IR_PIN, INPUT); 
  fan_servo.attach(BALL_RELEASE_SERVO_PIN);
  delay(100);
  fan_servo.write(BALL_RELEASE_SERVO_INITIAL_VAL);
  delay(1000);
}

void setup_consts(){
  x_line_robot = ROBOT_INITIAL_X;
  x_robot = ROBOT_INITIAL_X * LINE_SEP_DIST;
  y_line_robot = ROBOT_INITIAL_Y;
  y_robot = ROBOT_INITIAL_Y * LINE_SEP_DIST;
  robot_orient = ROBOT_INITIAL_ORIENT;
}

void main_setup(){
  setup_consts();
  int error_check; // Check if any routines don't run as expected. Return 0 if Routine runs error free
  init_hoppers(); // create structures for the hoppers
  error_check = detect_hoppers(); // Call routine to find position and orientation of hoppers
  if (error_check != 0){ detect_hoppers_error(error_check);} // In event of failure, call backup routine
  error_check = get_first_ball(); // code for getting the first ball from a random hopper
  if (error_check != 0) get_first_ball_error(error_check);
}

void main_loop(){
  int error_check; // Check if any routines don't run as expected. Return 0 if Routine runs error free
  robot_go_to_gameboard();
  error_check = pre_board_scan();
  if (error_check != 0){ pre_board_scan_error(error_check);}
  place_game_ball();
  robot_go_to_gameboard();
  error_check = post_board_scan();
  if (error_check != 0){ post_board_scan_error(error_check);}
  go_to_next_hopper();
  error_check = orient_on_hopper();
  if (error_check != 0){ orient_on_hopper_error(error_check);}
  pick_up_game_ball();
  error_check = go_back_to_line();
  if (error_check != 0){ go_back_to_line_error(error_check);}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        ROBOT MOVEMENT
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

void start_robot_clockwise_align(){
  robot_direction = 2;
  analogWrite(RIGHT_WHEEL_ENABLE_PIN, RIGHT_WHEEL_ALIGN_SPEED);
  analogWrite(LEFT_WHEEL_ENABLE_PIN, LEFT_WHEEL_ALIGN_SPEED);
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

void start_robot_counterclockwise_align(){
  robot_direction = -2;
  analogWrite(RIGHT_WHEEL_ENABLE_PIN, RIGHT_WHEEL_ALIGN_SPEED);
  analogWrite(LEFT_WHEEL_ENABLE_PIN, LEFT_WHEEL_ALIGN_SPEED);
  digitalWrite(LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
}

void start_robot_clockwise_orient(){
  robot_direction = 2;
  analogWrite(RIGHT_WHEEL_ENABLE_PIN, RIGHT_WHEEL_ORIENT_SPEED);
  analogWrite(LEFT_WHEEL_ENABLE_PIN, LEFT_WHEEL_ORIENT_SPEED);
  digitalWrite(LEFT_BACKWARD_PIN, LOW);
  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_FORWARD_PIN, LOW);
}

void start_robot_counterclockwise_orient(){
  robot_direction = -2;
  analogWrite(RIGHT_WHEEL_ENABLE_PIN, RIGHT_WHEEL_ORIENT_SPEED);
  analogWrite(LEFT_WHEEL_ENABLE_PIN, LEFT_WHEEL_ORIENT_SPEED);
  digitalWrite(LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(LEFT_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
}

void robot_quarter_turn_clockwise(){ //the robot turns 90 degrees clockwise, must be starting on a line
  align_robot(2);
  start_robot_clockwise();
  unsigned long cur_time = micros();
  int check = 1;
  while (check != 0){ //need to get off of the initial line
    check = check_line_sensors_on_line(cur_time); 
    cur_time = micros();
  }
  while (check != 1){
    check = check_line_sensors_on_line(cur_time); //keep turning until the line sensors all on a line
    cur_time = micros();
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
  normalize_orient(); //put orientation between -pi and pi
  float new_orient = -1 * PI;
  while (new_orient <= robot_orient + EPSILON){
    new_orient += PI_OVER_TWO;
  }
  robot_orient = new_orient;
  normalize_orient();
  start_robot_counterclockwise_align();
  delay(TURN_CORRECTION_DELAY);
  align_robot(0);
}

void robot_quarter_turn_counterclockwise(){ //the robot turns 90 degrees counterclockwise, must be starting on a line
  align_robot(2);
  start_robot_counterclockwise();
  delay(300);
  int check = 1;
  unsigned long cur_time = micros();
  while (check != 0){ //need to get off of the initial line, checks for 2 in a row to prevent error
    check = check_line_sensors_on_line(cur_time); 
    cur_time = micros();
  }
  while (check != 1){
    check = check_line_sensors_on_line(cur_time); //keep turning until the line sensors all on a line
    cur_time = micros();
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
  normalize_orient(); //put orientation between -pi and pi
  float new_orient = PI;
  while (new_orient > robot_orient - EPSILON){
    new_orient -= PI_OVER_TWO;
  }
  robot_orient = new_orient;
  normalize_orient();
  start_robot_clockwise_align();
  delay(TURN_CORRECTION_DELAY);
  align_robot(1);
}

void robot_turn_up(){ //turns robot so that it is facing direction 0
  normalize_orient();
  if (robot_orient >= PI_OVER_TWO + ROBOT_TURN_UP_ERROR) robot_quarter_turn_counterclockwise();
  if (robot_orient <= -1 * PI_OVER_TWO - ROBOT_TURN_UP_ERROR) robot_quarter_turn_clockwise();
  if (robot_orient >= ROBOT_TURN_UP_ERROR) robot_quarter_turn_counterclockwise();
  if (robot_orient <=  -1 * ROBOT_TURN_UP_ERROR) robot_quarter_turn_clockwise();
  align_robot(2);
}

void robot_turn_right(){ //turns robot so that it is facing direction PI/2
  normalize_orient();
  if (robot_orient <= -1 * ROBOT_TURN_UP_ERROR && robot_orient >= -1 * PI_OVER_TWO - ROBOT_TURN_UP_ERROR) robot_quarter_turn_clockwise();
  if (robot_orient >= -2 * PI - ROBOT_TURN_UP_ERROR && robot_orient <= -1 * PI_OVER_TWO - ROBOT_TURN_UP_ERROR) robot_quarter_turn_counterclockwise();
  if (robot_orient <= PI_OVER_TWO - ROBOT_TURN_UP_ERROR) robot_quarter_turn_clockwise();
  if (robot_orient >= PI_OVER_TWO + ROBOT_TURN_UP_ERROR || robot_orient <= -2 * PI + ROBOT_TURN_UP_ERROR) robot_quarter_turn_counterclockwise();
  align_robot(2);
}

void robot_turn_down(){ //turns robot so that it is facing direction PI or -PI
  normalize_orient();
  if (robot_orient <= PI_OVER_TWO - ROBOT_TURN_UP_ERROR) robot_quarter_turn_clockwise();
  if (robot_orient >= -1 * PI_OVER_TWO + ROBOT_TURN_UP_ERROR) robot_quarter_turn_counterclockwise();
  if (robot_orient <= PI - ROBOT_TURN_UP_ERROR) robot_quarter_turn_clockwise();
  if (robot_orient >=  -1 * PI - ROBOT_TURN_UP_ERROR) robot_quarter_turn_clockwise();
  align_robot(2);
}

void robot_turn_left(){ //turns robot so that it is facing direction -PI/2
  normalize_orient();
  if (robot_orient >= ROBOT_TURN_UP_ERROR && robot_orient <= PI_OVER_TWO + ROBOT_TURN_UP_ERROR) robot_quarter_turn_counterclockwise();
  if (robot_orient <= PI - ROBOT_TURN_UP_ERROR && robot_orient >= PI_OVER_TWO - ROBOT_TURN_UP_ERROR) robot_quarter_turn_clockwise();
  if (robot_orient >=  -1 * PI_OVER_TWO + ROBOT_TURN_UP_ERROR) robot_quarter_turn_counterclockwise();
  if (robot_orient <= -1 * PI_OVER_TWO - ROBOT_TURN_UP_ERROR || robot_orient >= PI - ROBOT_TURN_UP_ERROR) robot_quarter_turn_clockwise();
  align_robot(2);
}

int check_slightly_right(unsigned long cur_time){ //return 1 if left sensor is HIGH
  if (cur_time >= line_sensor_time){
    update_line_sensor_vals(cur_time);
    if (line_sensor_checks == NUM_LINE_SENSORS){
      /*Serial.print(left_line_sensor_val);
  Serial.print('\t');
  Serial.print(mid_line_sensor_val);
  Serial.print('\t');
  Serial.print(right_line_sensor_val);
  Serial.print('\t');
  Serial.print(main_line_sensor_val);
  Serial.print('\t');
  Serial.println(side_line_sensor_val);*/
      line_sensor_state = 0; //ready for new check
      if (left_line_sensor_val == HIGH && right_line_sensor_val == LOW) return 1;
      return 0;
    }
    return 2;
  }
  return 2; //didnt check
}

int check_slightly_left(unsigned long cur_time){ //return 1 if right sensor is HIGH
  if (cur_time >= line_sensor_time){
    update_line_sensor_vals(cur_time);
    if (line_sensor_checks == NUM_LINE_SENSORS){
      /*Serial.print(left_line_sensor_val);
  Serial.print('\t');
  Serial.print(mid_line_sensor_val);
  Serial.print('\t');
  Serial.print(right_line_sensor_val);
  Serial.print('\t');
  Serial.print(main_line_sensor_val);
  Serial.print('\t');
  Serial.println(side_line_sensor_val);*/
      line_sensor_state = 0; //ready for new check
      if (right_line_sensor_val == HIGH && left_line_sensor_val == LOW) return 1;
      return 0;
    }
    return 2;
  }
  return 2; //didnt check
}

void align_robot(int prev_direct){ //aligns robot on to line
  int check = 2;
  unsigned long cur_time = micros();
  line_sensor_time = cur_time;
  delay(PULSE_LINE_SENSOR_TIME/1000);
  line_sensor_state = 0;
  while (check != 1 and prev_direct != 2){
    check = check_line_sensors_on_line(cur_time);
    if (check == 0){
      if (prev_direct == 0){ // 0 is we were turning clockwise
        start_robot_counterclockwise_align();
      }
      else {
        start_robot_clockwise_align();
      }
    }
    cur_time = micros();
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
  check= 1;
  while (check != 0){ //rotated slightly too much clockwise
    check = check_slightly_right(cur_time);
    cur_time = micros();
    start_robot_counterclockwise_align();
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
  check = 1;
  while (check != 0){ //rotated slightly too much counterclockwise
    cur_time = micros();
    check = check_slightly_left(cur_time);  
    start_robot_clockwise_align();
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
}

void update_line_sensors(){ //update every line sensor val
  line_sensor_checks = 0;
  line_sensor_state = 0;
  unsigned long cur_time = micros();
  line_sensor_time = cur_time + PULSE_LINE_SENSOR_TIME;
  while (1){
    cur_time = micros();
    if (line_sensor_time > cur_time) continue;
    update_line_sensor_vals(cur_time);
    if (line_sensor_checks == NUM_LINE_SENSORS) return;
  }
}

void robot_drive_til_line(int line, int direct){ //line is the line number to reach, direct is 0 for x, 1 for y
  unsigned long cur_time = micros();
  while (1){
    cur_time = micros();
    check_line_sensors(cur_time);
    if (x_line_robot == line && direct == 0) break;
    if (y_line_robot == line && direct == 1) break;
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
  left_wheel_speed = LEFT_WHEEL_ENCODER_SPEED;
  right_wheel_speed = RIGHT_WHEEL_ENCODER_SPEED;
  int check = 0;
  while (check != 1){
    cur_time = micros();
    check = check_line_sensors_on_line(cur_time); //keep turning until the line sensors all on a line
    if (check == 0) start_robot_backward();
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
}

void robot_go_to_location(int final_x, int final_y){
  normalize_orient(); //forces angle between -pi and pi
  if (y_line_robot < 2){ //need to go up first
    robot_turn_up();
    robot_drive_vertic(2); //drive to second vertical line
  }
  if (final_y == y_line_robot || final_x == x_line_robot){
    robot_drive_horiz(final_x);
    robot_drive_vertic(final_y);
  }
  else{
    if (y_line_robot > MID_ZONE_TOP_Y || y_line_robot < MID_ZONE_BOT_Y){ //only need to go horizontally and vertically
      if (abs(robot_orient) < LINE_PASS_ANGLE_ERROR && final_y > y_line_robot){
        robot_drive_vertic(final_y);
        robot_drive_horiz(final_x);
        return;
      }
      if (abs(robot_orient) > PI - LINE_PASS_ANGLE_ERROR && final_y < y_line_robot){
        robot_drive_vertic(final_y);
        robot_drive_horiz(final_x);
      }
      else {
        robot_drive_horiz(final_x);
        robot_drive_vertic(final_y);
      }
    }
    else{ //need to go around the middle
      if (final_x < GAMEFIELD_RED_LINE_NUM){ //go left to line x=2 then to correct y then to correct x
        robot_drive_horiz(ROBOT_FOLLOW_LEFT_COLUMN_VAL);
        robot_drive_vertic(final_y);
        robot_drive_horiz(final_x);
      }
      else{
        robot_drive_horiz(ROBOT_FOLLOW_RIGHT_COLUMN_VAL);
        robot_drive_vertic(final_y);
        robot_drive_horiz(final_x);
      }
    }
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
}

void robot_go_to_gameboard(){
  robot_go_to_location(GAMEBOARD_INITIAL_X_ROBOT, GAMEBOARD_INITIAL_Y_ROBOT);
  robot_turn_up();
  unsigned long cur_time = micros();
  send_hopper_orient_ping(cur_time);
  start_robot_forward();
  while (front_ping_dist + y_robot < RELEASE_BALL_Y_VALUE){
    check_line_sensors(cur_time);
    if (front_ping_time < cur_time){
      send_hopper_orient_ping(cur_time);
    }
    cur_time = micros();
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
  robot_quarter_turn_counterclockwise();
}

void robot_go_to_hopper(){ //find the location of the next hopper and go to it
  update_next_hopper();
}

void update_next_hopper(){ //finds the best hopper to go to
  for (int x = 0; x < NUMBER_OF_HOPPERS; x++){
    if (hoppers[hopper_order[x]].balls > 0){
      next_hopper = hopper_order[x];
      return;
    }
  }
}

void go_to_next_hopper(){ //drive to optimal position for hopper
  if (hoppers[next_hopper].orient == 0) robot_go_to_location(hoppers[next_hopper].x, hoppers[next_hopper].y + 2); //go to location 2 above the hopper
  if (hoppers[next_hopper].orient == 1) robot_go_to_location(hoppers[next_hopper].x + 1, hoppers[next_hopper].y - 1); //go to location 2 below the hopper
  if (hoppers[next_hopper].orient == 2) robot_go_to_location(1, 2); //go to location up and right of the bottom left hopper
  if (hoppers[next_hopper].orient == 3) robot_go_to_location(7, 2); //go to location up and left of the bottom right hopper
}

void robot_drive_horiz(int line){ //robot drives to the line specified by line
  normalize_orient();
  if (line == x_line_robot) return;
  if (line > x_line_robot){ //need to go right
      while (abs(robot_orient - PI_OVER_TWO) > LINE_PASS_ANGLE_ERROR){ //not facing right
        if (robot_orient < PI_OVER_TWO && robot_orient >= -1 * PI_OVER_TWO - EPSILON) robot_quarter_turn_clockwise();
        else robot_quarter_turn_counterclockwise();
      }
      if(abs(robot_orient - PI_OVER_TWO) < LINE_PASS_ANGLE_ERROR){ //facing to right so drive forward
        start_robot_straight_forward();
      }
      else{ //facing to left so drive backwards
        start_robot_straight_backward();
      }
      robot_drive_til_line(line, 0);  //drive to line
    }
    else { //need to go left
      while (abs(robot_orient + PI_OVER_TWO) > LINE_PASS_ANGLE_ERROR){ //not facing left
        if (abs(robot_orient) > PI_OVER_TWO) robot_quarter_turn_clockwise();
        else robot_quarter_turn_counterclockwise();
      }
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
  normalize_orient();
  if (line == y_line_robot) return;
  if (line > y_line_robot){ //need to go up
      while (abs(robot_orient) > LINE_PASS_ANGLE_ERROR){ //not facing up/down
        if (robot_orient < 0) robot_quarter_turn_clockwise();
        else robot_quarter_turn_counterclockwise();
      }
      if(abs(robot_orient) < LINE_PASS_ANGLE_ERROR){ //facing up so drive forward
        start_robot_straight_forward();
      }
      else{ //facing down so drive backwards
        start_robot_straight_backward();
      }
      robot_drive_til_line(line, 1);  //drive to line
    }
    else { //need to go down
      while (robot_orient > LINE_PASS_ANGLE_ERROR - PI && robot_orient < PI - LINE_PASS_ANGLE_ERROR){ //not facing up/down
        if (robot_orient > 0) robot_quarter_turn_clockwise();
        else robot_quarter_turn_counterclockwise();
      }
      if(abs(abs(robot_orient) - PI) < LINE_PASS_ANGLE_ERROR){ //facing down so drive forward
        start_robot_straight_forward();
      }
      else{ //facing up so drive backwards
        start_robot_straight_backward();
      }
      robot_drive_til_line(line, 1);  //drive to left line
    }
}

void get_ball(){
  //get ball specified
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        LINE FOLLOWING
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    line_sensor_checks = 0; // number of line sensors we have checked
  }
  if (line_sensor_state == 1){
    left_line_sensor_val = digitalRead(LEFT_LINE_SENSOR_PIN);
  }
  if (line_sensor_state == 2){
    mid_line_sensor_val = digitalRead(MID_LINE_SENSOR_PIN);
  }
  if (line_sensor_state == 3){
    right_line_sensor_val = digitalRead(RIGHT_LINE_SENSOR_PIN);
  }
  if (line_sensor_state == 4){
    main_line_sensor_val = digitalRead(MAIN_LINE_SENSOR_PIN);
  }
  if (line_sensor_state == 5){
    side_line_sensor_val = digitalRead(SIDE_LINE_SENSOR_PIN);
  }
  if (line_sensor_checks < NUM_LINE_SENSORS){
    line_sensor_time = cur_time + line_sensor_delay[line_sensor_checks];
    line_sensor_state = line_sensor_order[line_sensor_checks];
  }
  else{
    line_sensor_state = 0;
    line_sensor_time = cur_time + PULSE_LINE_SENSOR_TIME;
  }
  line_sensor_checks += 1;
}


void check_line_following(unsigned long cur_time){
  /*Serial.print(left_line_sensor_val);
  Serial.print('\t');
  Serial.print(mid_line_sensor_val);
  Serial.print('\t');
  Serial.print(right_line_sensor_val);
  Serial.print('\t');
  Serial.print(main_line_sensor_val);
  Serial.print('\t');
  Serial.println(side_line_sensor_val);*/
  //
  /*Serial.print(cur_time);
  Serial.print('\t');
  Serial.print(side_line_sensor_val);
  Serial.print('\t');
  Serial.print(y_line_robot);
  Serial.print('\t');
  Serial.println(robot_on_line);*/
  //if (side_line_sensor_val == HIGH && main_line_sensor_val == HIGH){ // robot passes a line
  if (side_line_sensor_val == HIGH){ // robot passes a line
    if (robot_on_line == 0){ 
      robot_passed_line(); //passed line
      robot_on_line = ROBOT_ON_LINE_ERROR;
    }
    else{ 
      robot_on_line = ROBOT_ON_LINE_ERROR;
    }
  }
  else {
    if (robot_on_line > 0) robot_on_line -= 1;
  }
  correction_factor += correction_state;
  if (left_line_sensor_val == LOW && mid_line_sensor_val == HIGH && right_line_sensor_val == LOW && main_line_sensor_val == HIGH){ //on track
    if (robot_direction == 1){
      line_on_track_forward(cur_time);
    }
    if (robot_direction == -1){
      line_on_track_backward(cur_time);
    }
  }
  if (left_line_sensor_val == HIGH && mid_line_sensor_val == HIGH && right_line_sensor_val == LOW){ //slightly too far right
    if (robot_direction == 1){
      line_slightly_right_forward();
    }
    if (robot_direction == -1){
      line_slightly_right_backward();
    }
  }
  if (left_line_sensor_val == HIGH && mid_line_sensor_val == LOW && right_line_sensor_val == LOW){ //too far right
    if (robot_direction == 1){
      line_far_right_forward();
    }
    if (robot_direction == -1){
      line_far_right_backward();
    }
  }
  if (right_line_sensor_val == HIGH && mid_line_sensor_val == HIGH && left_line_sensor_val == LOW){ //slightly too far left
    if (robot_direction == 1){
      line_slightly_left_forward();
    }
    if (robot_direction == -1){
      line_slightly_left_backward();
    }
  }
  if (right_line_sensor_val == HIGH && mid_line_sensor_val == LOW && left_line_sensor_val == LOW){ //too far left
    if (robot_direction == 1){
      line_far_left_forward();
    }
    if (robot_direction == -1){
      line_far_left_backward();
    }
  }
  if (right_line_sensor_val == LOW && mid_line_sensor_val == LOW && left_line_sensor_val == LOW){//robot is lost!!!!
    robot_is_lost();
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
  if (robot_orient >= 2 * PI - LINE_PASS_ANGLE_ERROR || robot_orient <= LINE_PASS_ANGLE_ERROR){ //robot is facing upwards
    y_line_robot += robot_direction;
    y_robot = y_line_robot * LINE_SEP_DIST;
  }
  if (abs(robot_orient - PI_OVER_TWO) <= LINE_PASS_ANGLE_ERROR){ //robot is facing right
    x_line_robot += robot_direction;
    if (x_line_robot == GAMEFIELD_RED_LINE_NUM) x_line_robot += robot_direction;
    x_robot = x_line_robot * LINE_SEP_DIST;
  }
  if (abs(robot_orient - PI) <= LINE_PASS_ANGLE_ERROR){ //robot is facing down
    y_line_robot -= robot_direction;
    y_robot = y_line_robot * LINE_SEP_DIST;
  }
  if (abs(robot_orient - PI_THREE_OVER_TWO) <= LINE_PASS_ANGLE_ERROR){ //robot is facing left
    x_line_robot -= robot_direction;
    if (x_line_robot == GAMEFIELD_RED_LINE_NUM) x_line_robot -= robot_direction;
    x_robot = x_line_robot * LINE_SEP_DIST;
  }
}

void line_on_track_forward(unsigned long cur_time){ //back on track so drive straight again
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED;
  correction_state = 0;
  correction_factor = 0;
}

void line_slightly_left_forward(){ //turn robot right
  correction_state = 1;
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED - MIN_SPEED_CORRECTION_VALUE;
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED;
}

void line_slightly_right_forward(){ //turn robot left
  correction_state = -1;
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED - MIN_SPEED_CORRECTION_VALUE;
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
}

void line_far_left_forward(){ //too far left so turn right
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED - MAX_SPEED_CORRECTION_VALUE;
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED;
  correction_state = 2;
}

void line_far_right_forward(){ //too far right so turn left
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED - MAX_SPEED_CORRECTION_VALUE;
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
  correction_state = -2;
}

void line_on_track_backward(unsigned long cur_time){ //back on track so drive straight again
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

void line_slightly_left_backward(){
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
    right_wheel_speed = RIGHT_WHEEL_MAX_SPEED - MIN_SPEED_CORRECTION_VALUE;
    left_wheel_speed = LEFT_WHEEL_MAX_SPEED;
  }
}

void line_slightly_right_backward(){
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

void line_far_left_backward(){ //too far left
  off_track_flag = 1;
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED - MAX_SPEED_CORRECTION_VALUE;
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED;
  correction_state = 2;
  if (correction_factor < 0) correction_factor = 2; //robot thinks its angled right, so reset this so it will think its angled left
}

void line_far_right_backward(){
  off_track_flag = 1;
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED - MAX_SPEED_CORRECTION_VALUE;
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
  correction_state = -2; 
  if (correction_factor > 0) correction_factor = -2; //robot thinks its angled left, so reset this so it will think its angled right
}

void robot_is_lost(){ //robot is lost
  return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        POSITION TRACKING+ENCODER
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void update_location(unsigned long cur_time){
  /*Serial.println(wheel_ir_time);*/
  if (cur_time > wheel_ir_time){
    int right_ir = analogRead(RIGHT_IR_PIN);
    if (right_ir < IR_DIFFERENCE_VAL) right_ir = 0;
    if (right_ir >= IR_DIFFERENCE_VAL) right_ir = 1;
    /*Serial.print(right_ir);
    Serial.print('\t');
    Serial.println(y_robot);*/
    //int left_ir = digitalRead(LEFT_IR_PIN); //not used
    if (robot_direction == 1 || robot_direction == -1){ //robot is driving straight forward or backwards
      if (prev_ir_right == 0 && right_ir == 1){
        x_robot += sin(robot_orient) * robot_direction * WHEEL_CIRCUMFERENCE / WHEEL_NUM_HOLES;
        y_robot += cos(robot_orient) * robot_direction * WHEEL_CIRCUMFERENCE / WHEEL_NUM_HOLES;
      }
    }
    if (robot_direction == 2){ //robot is turning clockwise
      if (prev_ir_right == 0 && right_ir == 1){
        robot_orient += (WHEEL_CIRCUMFERENCE / WHEEL_NUM_HOLES) / ROBOT_TURNING_RADIUS;
      }
    }
    if (robot_direction == -2){ //robot is turning counterclockwise
      if (prev_ir_right == 0 && right_ir == 1){
        robot_orient -= (WHEEL_CIRCUMFERENCE / WHEEL_NUM_HOLES) / ROBOT_TURNING_RADIUS;
      }
    }
    wheel_ir_time = cur_time + WHEEL_IR_DELAY;
    prev_ir_right = right_ir;
    //prev_ir_left == left_ir;
  }
}

int check_line_sensors_on_line(unsigned long cur_time){ //return 1 if SIDE line sensors on line, return 0 if not, return 2 if not updated yet
  if (cur_time >= line_sensor_time){
    update_line_sensor_vals(cur_time);
    if (line_sensor_checks == NUM_LINE_SENSORS){
      line_sensor_state = 0; //ready for new check
      /*Serial.print(left_line_sensor_val);
      Serial.print('\t');
      Serial.print(mid_line_sensor_val);
      Serial.print('\t');
      Serial.print(right_line_sensor_val);
      Serial.print('\t');
      Serial.print(main_line_sensor_val);
      Serial.print('\t');
      Serial.println(side_line_sensor_val);*/
      if (side_line_sensor_val == HIGH) return 1;
      return 0;
    }
    return 2;
  }
  return 2; //didnt check
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        HOPPER DETECTION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void send_hopper_detect_ping(unsigned long cur_time){
  unsigned int ping_record_time = hopper_detector.ping(); //measures time to receive ping
  ping_dist = ping_record_time / US_ROUNDTRIP_CM + (int) y_robot + HOP_DETECT_Y_OFFSET; // ping distance from gamefield bottom
  Serial.println(ping_dist);
  ping_time = cur_time + HOP_DETECT_PING_DELAY; // add delay before another ping is sent
}

int detect_hoppers(){
  //hopper_detect_initial_pos(); // get to initial position for detecting hoppers
  x_line_robot = 6;
  x_robot = 60;
  y_line_robot = 1;
  y_robot = 10;
  robot_orient = -1 * PI_OVER_TWO;
  start_robot_straight_forward();
  unsigned long cur_time = micros(); //Record the current robot time
  ping_time = cur_time; // initialize ping time
  if (wheel_ir_time == 0){
    wheel_ir_time = cur_time;}
  int prev_dist = 0; // Saves last measured important distance
  int cur_hopper = 0; //record current hopper we are finding, either 0 or 1
  int flag = 1; // 1 if we are ready to detect a new hopper, 2 if we found 3rd leg, else 0
  ping_dist = 0; // keeps track of the distance measured by the most recent ping
  int prev_dist2 = 0; //keeps track of previous ping
  int count = 0; // keeps track of the number of similar distance pings in a row
  while (1){ //Run until end is reached
    cur_time = micros(); //update current time every loop
    update_location(cur_time); // updates x_robot and y_robot
    check_line_sensors(cur_time);
    if (cur_time > ping_time){ // send another ping if enough time has passed
    Serial.print(ping_dist);
    Serial.print('\t');
      send_hopper_detect_ping(cur_time);
      
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
        if (ping_dist <prev_dist - HOPPER_DETECT_ERROR && ping_dist -(int)y_robot - HOP_DETECT_Y_OFFSET > HOPPER_DETECT_Y_ERROR && count >= HOPPER_VALUES_IN_ROW) flag = 2; //found3rdleg
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
      stop_robot_motion();
      delay(1000);
      start_robot_forward();
        hoppers[cur_hopper].x = ((int) x_robot + HOP_DETECT_X_OFFSET + HOP_DETECT_X_ERROR) / LINE_SEP_DIST;
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
        stop_robot_motion();
      delay(1000);
      start_robot_forward();
        hoppers[cur_hopper].x = ((int) x_robot + HOP_DETECT_X_OFFSET + HOP_DETECT_X_ERROR) / LINE_SEP_DIST;
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
  stop_robot_motion();
  display_hoppers(); //Shows data for each hopper on serial
  display_hoppers2(); //Shows data for each hopper on serial
  set_hopper_order();
  return 0;
}

void display_hoppers2(){
  while (1){
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    delay(2000);
    for (int x = 0; x < hoppers[0].x; x++){
      digitalWrite(13, LOW);
      delay(300);
      digitalWrite(13, HIGH);
      delay(300);
    }
    digitalWrite(13, LOW);
    delay(2000);
    for (int x = 0; x < hoppers[0].y; x++){
      digitalWrite(13, LOW);
      delay(300);
      digitalWrite(13, HIGH);
      delay(300);
    }
    digitalWrite(13, LOW);
    delay(2000);
    for (int x = 0; x < hoppers[0].orient; x++){
      digitalWrite(13, LOW);
      delay(300);
      digitalWrite(13, HIGH);
      delay(300);
    }
    digitalWrite(13, LOW);
    delay(2000);
    for (int x = 0; x < 30; x++){
      digitalWrite(13, LOW);
      delay(100);
      digitalWrite(13, HIGH);
      delay(100);
    }
    for (int x = 0; x < hoppers[1].x; x++){
      digitalWrite(13, LOW);
      delay(300);
      digitalWrite(13, HIGH);
      delay(300);
    }
    digitalWrite(13, LOW);
    delay(2000);
    for (int x = 0; x < hoppers[1].y; x++){
      digitalWrite(13, LOW);
      delay(300);
      digitalWrite(13, HIGH);
      delay(300);
    }
    digitalWrite(13, LOW);
    delay(2000);
    for (int x = 0; x < hoppers[1].orient; x++){
      digitalWrite(13, LOW);
      delay(300);
      digitalWrite(13, HIGH);
      delay(300);
    }
    digitalWrite(13, LOW);
    delay(2000);
    for (int x = 0; x < 30; x++){
      digitalWrite(13, LOW);
      delay(100);
      digitalWrite(13, HIGH);
      delay(100);
    }
    digitalWrite(13, HIGH);
    delay(3000);
  }
}

void display_hoppers(){
  stop_robot_motion(); //turn off wheel motors
  delay(STOP_MOTION_DELAY);
  if (SERIAL_COMM_BOOL || 1){
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


void detect_hoppers_error(int error){
  // Set error flags so that robot assumes just one random hopper in middle, but don't drive through middle error
}

void hopper_detect_initial_pos(){
  // drive the robot to a position on left side of board just outside of the starting area
  // update current location
  start_robot_forward();
  unsigned long cur_time = micros();
  while ((x_robot - HOPPER_DETECT_INITIAL_X) > POSITION_ERROR){
    check_line_sensors(cur_time);
    update_location(cur_time);
    cur_time = micros();
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
  normalize_orient();
  while (abs(robot_orient + PI_OVER_TWO) > LINE_PASS_ANGLE_ERROR){ //not facing left
        if (abs(robot_orient) > PI_OVER_TWO) robot_quarter_turn_clockwise();
        else robot_quarter_turn_counterclockwise();
      }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
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

void set_hopper_order(){ //gets the best hopper order: upmost correct orientation mid hopper > corner > wrong orientation mid hopper and left > right
  if (hoppers[0].orient == 1 && hoppers[1].orient == 1){ //both are good orientation
    if (hoppers[0].y >= hoppers[1].y){ //hopper 0 is farther up so it is better
      for (int x = 0; x < NUMBER_OF_HOPPERS; x++){
        hopper_order[x] = x;
      }
    }
    else{ //hopper 0 is farther down so it is worse
      hopper_order[0] = 1;
      hopper_order[1] = 0;
      hopper_order[2] = 2;
      hopper_order[3] = 3;
    }
    return;
  }
  if (hoppers[0].orient == 1){
    hopper_order[0] = 0;
    hopper_order[1] = 2;
    hopper_order[2] = 3;
    hopper_order[3] = 1;
    return;
  }
  if (hoppers[1].orient == 1){
    hopper_order[0] = 1;
    hopper_order[1] = 2;
    hopper_order[2] = 3;
    hopper_order[3] = 0;
    return;
  }
  hopper_order[0] = 2;
  hopper_order[1] = 3;
  if (hoppers[0].y >= hoppers[1].y){
    hopper_order[2] = 0;
    hopper_order[3] = 1;
  }
  else{
    hopper_order[2] = 1;
    hopper_order[3] = 0;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        SERIAL COMMUNICATION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    Serial.print(x_robot);
    Serial.print(',');
    Serial.print(y_robot);
    Serial.print(',');
    Serial.print(robot_orient);
    Serial.print(',');
    Serial.print(next_ball_column);
    Serial.print(',');
    Serial.print(micros());
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        PICK UP GAMEBALL
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void send_hopper_orient_ping(unsigned long cur_time){
  unsigned int ping_record_time = ball_grab_detector.ping(); //measures time to receive ping
  front_ping_dist = ping_record_time / US_ROUNDTRIP_CM + ROBOT_LENGTH_FRONT; // ping distance from middle of robot
  front_ping_time = cur_time + BALL_GRAB_PING_DELAY; // add delay before another ping is sent
  if (front_ping_dist > BALL_GRAB_MAX_DIST || front_ping_dist == ROBOT_LENGTH_FRONT) front_ping_dist = 0;
  Serial.println(front_ping_dist);
}

int orient_on_hopper(){
  normalize_orient();
  if (hoppers[next_hopper].orient == 0) robot_turn_up(); //orient robot based on which hopper it is at
  if (hoppers[next_hopper].orient == 1) robot_turn_down();
  if (hoppers[next_hopper].orient == 2) robot_turn_down();
  if (hoppers[next_hopper].orient == 3) robot_turn_right();
  orient_to_hopper(); //get right angle to hopper
  align_to_hopper(); //get right distance to hopper
}

void orient_to_hopper(){
  start_robot_clockwise_orient();
  unsigned long cur_time = micros();
  send_hopper_orient_ping(cur_time);
  float prev_val = 0;
  while (front_ping_dist != 0){
    if (front_ping_time < cur_time){
      send_hopper_orient_ping(cur_time);
      prev_val = front_ping_dist;
    }
    cur_time = micros();
  }
  while(front_ping_dist < prev_val + 3){
    if (front_ping_time < cur_time) { 
      send_hopper_orient_ping(cur_time);
    }
    cur_time = micros();
  }
  int prev_dists[NUMBER_OF_HOP_ORIENT_DISTS] = {0};
  prev_dists[0] = front_ping_dist;
  for (int x = 1; x < NUMBER_OF_HOP_ORIENT_DISTS; x++){
    while (1){ //get additional prev_dist value to average them
      if (front_ping_time < cur_time){ 
        if (front_ping_dist == 0) continue; //prevents error values
        send_hopper_orient_ping(cur_time);
        cur_time = micros();
        break;
      }
      cur_time = micros();
    }
    prev_dists[x] = front_ping_dist;
  }
  int avg_prev_dist = 0;
  int avg_cur_dist = 0;
  while (avg_prev_dist >= avg_cur_dist){
    if (front_ping_time < cur_time){
      /*Serial.print(avg_prev_dist);
    Serial.print('\t');
    Serial.println(avg_cur_dist);*/
      send_hopper_orient_ping(cur_time);
      if (front_ping_dist == 0) continue; //prevents error values
      for (int x = 0; x < NUMBER_OF_HOP_ORIENT_DISTS - 1; x++){
        prev_dists[x] = prev_dists[x+1];
      }
      prev_dists[NUMBER_OF_HOP_ORIENT_DISTS - 1] = front_ping_dist;
      avg_prev_dist = 0;
      avg_cur_dist = 0;
      for (int x = 0; x < NUMBER_OF_HOP_ORIENT_DISTS / 2; x++){ //find average distances
        avg_prev_dist += prev_dists[x];
        avg_cur_dist += prev_dists[NUMBER_OF_HOP_ORIENT_DISTS / 2 + x];
      }
    }
   cur_time = micros(); 
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
}
void align_to_hopper(){
  left_wheel_speed = LEFT_WHEEL_ORIENT_SPEED;
  right_wheel_speed = RIGHT_WHEEL_ORIENT_SPEED;
  int distances[NUMBER_OF_HOP_ORIENT_DISTS / 2] = {0};
  distances[0] = front_ping_dist;
  unsigned long cur_time = micros();
  int x = 1;
  while (x < NUMBER_OF_HOP_ORIENT_DISTS / 2){
    if (front_ping_time <= cur_time){
      send_hopper_orient_ping(cur_time);
      if (front_ping_dist == 0) continue; //prevents error values
      distances[x] = front_ping_dist;
      x++;
    }
    cur_time = micros();
  }
  //check if at right distance
  float avg_dist = 0.0;
  while (abs(avg_dist - BALL_GRAB_PROPER_DISTANCE) > BALL_GRAB_DISTANCE_ERROR){ //get within close distance of hopper
    while (1){
      if (front_ping_time <= cur_time){
        send_hopper_orient_ping(cur_time);
        if (front_ping_dist == 0) continue; //prevents error values
        for (x = 0; x < NUMBER_OF_HOP_ORIENT_DISTS / 2 - 1; x++){
          distances[x] = distances[x + 1];
        }
        distances[NUMBER_OF_HOP_ORIENT_DISTS / 2 - 1] = front_ping_dist;
        break;
      }
      cur_time = micros();
    }
    avg_dist = 0.0;
    for (x = 0; x < NUMBER_OF_HOP_ORIENT_DISTS / 2; x++){ //find average
      avg_dist += distances[x];
    }
    avg_dist /= (NUMBER_OF_HOP_ORIENT_DISTS / 2);
    if (avg_dist > BALL_GRAB_PROPER_DISTANCE) start_robot_forward(); //too far away so go closer
    else start_robot_backward(); //too close so go backwards
  }  
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
}

void orient_on_hopper_error(int error){
  return;
}

void pick_up_game_ball(){
  //fan_servo.write(BALL_RELEASE_SERVO_INITIAL_VAL);
  digitalWrite(BALL_GRAB_FAN_PIN, LOW); //low means the fan is sucking up the ball
  delay(BALL_SUCTION_ON_TIME);
  digitalWrite(BALL_GRAB_FAN_PIN, HIGH);
  delay(BALL_SUCTION_OFF_TIME);
}

int go_back_to_line(){ //after getting a gameball, go back to the line
  update_line_sensors();
  start_robot_backward();
  while(main_line_sensor_val == LOW){
    update_line_sensors();
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
  //robot_quarter_turn_counterclockwise();
  return 0;
}

void go_back_to_line_error(int error_check){
 return;
} 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        PLACE GAMEBALL
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void place_game_ball(){
  left_wheel_speed = LEFT_WHEEL_GAMEBOARD_SPEED;
  right_wheel_speed = RIGHT_WHEEL_GAMEBOARD_SPEED;
  start_robot_backward();
  //if (abs(TEST_THREE_INITIAL_ORIENT - PI_OVER_TWO) < LINE_PASS_ANGLE_ERROR) start_robot_forward();
  //if(abs(TEST_THREE_INITIAL_ORIENT - PI_THREE_OVER_TWO) < LINE_PASS_ANGLE_ERROR) start_robot_backward();
  while(abs(x_robot - column_locations[TEST_THREE_FINAL_COLUMN]) > BALL_RELEASE_POSITION_ERROR){
    unsigned long cur_time = micros();
    update_location(cur_time);
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
  release_ball();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        OTHER FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void normalize_orient(){ //put robot's orientation in range -pi to pi
  if (robot_orient < 0){
    while (robot_orient < -1 * PI) robot_orient += 2 * PI;
  }
  else{
    while (robot_orient > PI) robot_orient -= 2 * PI;
  }
}

int get_first_ball(){
  next_hopper = FIRST_HOPPER_NUM;
  go_to_next_hopper();
  return grab_ball();
}

void get_first_ball_error(int error){
  return;
}

int grab_ball(){
	// orient vacuum pump over hopper
	// turn on vacuum pump to grab ball
}

int update_ldr_vals(){
  int y = 0;
  int flag = 1;
  for (int x = A1; x < A7; x++){
    ldr_vals[y] = analogRead(x);
    y++;
    if (ldr_vals[y-1] < ldr_wall_vals[y-1]) flag = 0;
  }
  if (flag == 1) return 1;
  return 2;
}

int pre_board_scan(){
  left_wheel_speed = LEFT_WHEEL_GAMEBOARD_SPEED;
  right_wheel_speed = RIGHT_WHEEL_GAMEBOARD_SPEED;
  start_robot_forward();
  unsigned long cur_time = micros();
  int x = 8;
  int flag = 1;
  while (x_robot > END_OF_SCAN_X && x >= 0){
    update_location(cur_time);
    int check = update_ldr_vals();
    if (check == 1 && flag == 1){ // wall
      x--;
      flag = 0;
    }
    if (check == 2){ //update vals
      flag = 1;
      for (int y = 0; y < 7; y++){
        if (ldr_vals[y] < ldr_ball_vals[y]){
          gameboard[x][y] = 0; // 0 is no ball          
        }
        if (ldr_vals[y] >= ldr_ball_vals[y] && gameboard[x][y] == 0){
          gameboard[x][y] = 2; //their ball
        }
      }
    }
  }
  int error_check = game_strategy();
  place_game_ball();
  x_line_robot = (x_robot) / LINE_SEP_DIST;
  robot_drive_til_line(3, 0);
  if (error_check != 0){ game_strategy_error(error_check);}    
}

void pre_board_scan_error(int error){
	// handle errors
}

int game_strategy(){
	// find best column to place the ball into
  // save to next_ball_column global variable
  next_ball_column++;
  next_ball_column = next_ball_column % NUMBER_BOARD_COLUMNS;
  return 0;
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

void controlled_locomotion(){
  x_line_robot = TEST_ONE_INITIAL_X;
  y_line_robot = TEST_ONE_INITIAL_Y;
  x_robot = TEST_ONE_INITIAL_X * LINE_SEP_DIST;
  y_robot = TEST_ONE_INITIAL_Y * LINE_SEP_DIST;
  robot_orient = TEST_ONE_INITIAL_ORIENT;
  robot_go_to_location(TEST_ONE_FINAL_X, TEST_ONE_FINAL_Y);
}

void navigate_game_board(){ // DOESNT WORK
  pick_up_game_ball();
  x_line_robot = TEST_FOUR_INITIAL_X;
  y_line_robot = TEST_FOUR_INITIAL_Y;
  x_robot = 95;
  y_robot = TEST_FOUR_INITIAL_Y * LINE_SEP_DIST;
  robot_orient = -1 * PI_OVER_TWO;
  /*if (x_robot < GAME_FIELD_WIDTH){ //go to left position
    robot_go_to_location(GAME_BOARD_LEFT_X, GAME_BOARD_LEFT_Y);
  }
  else{ //go to right position
    robot_go_to_location(GAME_BOARD_RIGHT_X, GAME_BOARD_RIGHT_Y);
  }*/
  //robot_turn_up();
  left_wheel_speed = LEFT_WHEEL_GAMEBOARD_SPEED;
  right_wheel_speed = RIGHT_WHEEL_GAMEBOARD_SPEED;
  start_robot_forward();
  /*while (y_robot < RELEASE_BALL_Y_VALUE){//drive until right y distance
    unsigned long cur_time = micros();
    update_location(cur_time);
  }*/
  //stop_robot_motion();
  //delay(STOP_MOTION_DELAY);
  //robot_quarter_turn_counterclockwise(); //align robot so it is parallel to gameboard
  start_robot_forward();
  while (abs(x_robot - column_locations[3]) > BALL_RELEASE_POSITION_ERROR){//drive until right x distance
    unsigned long cur_time = micros();
    update_location(cur_time);
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
  release_ball();
}

void navigate_hopper(){
  
}
void move_around_obstacle(){
  
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        TEST
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

void test_line_following(){
  setup_consts();
  start_robot_forward();
  unsigned long cur_time = micros();
  while (1){
    check_line_sensors(cur_time);
    cur_time = micros();
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
}

void test_turning(){
  setup_consts();
  robot_orient = 0;
  y_line_robot = 1;
  align_robot(2);
  delay(300);
  robot_drive_vertic(3);
  align_robot(2);
  delay(2000);
  /*for (int x = 0; x < 4; x++){
    robot_quarter_turn_clockwise();
    delay(1000);
  }*/
  for (int x = 0; x < 4; x++){
    robot_quarter_turn_counterclockwise();
    delay(1000);
  }
}

void test_locomotion(){
  x_line_robot = 3;
  x_robot = 10;
  y_robot = 10;
  y_line_robot = 1;
  robot_orient = 0;
  for (int x = 0; x < 4; x++){
    robot_drive_vertic(3);
    robot_drive_horiz(1);
    robot_drive_vertic(1);
    robot_drive_horiz(3);
  }
  //robot_drive_vertic(5);
}

void test_encoder(){
  start_robot_straight_forward();
  unsigned long cur_time = micros();
  x_line_robot = 1;
  x_robot = 10;
  y_robot = 10;
  y_line_robot = 1;
  robot_orient = 0;
  while(y_robot < 50){
    update_location(cur_time);
    cur_time = micros();
  }
  stop_robot_motion();
  
}

void test_align(){
  while (1){
    Serial.println("started");
    align_robot();
    delay(3000);
  }
}
//
//
// PE3
//
//
void test_align_hopper(){
  orient_to_hopper(); //get right angle to hopper
  align_to_hopper(); //get right distance to hopper
}

void test_locomotion2(){
  x_line_robot = 3;
  x_robot = 30;
  y_robot = 10;
  y_line_robot = 1;
  robot_orient = 0;
  robot_drive_vertic(3);
  robot_drive_horiz(1);
  robot_orient = PI_OVER_TWO;
  stop_robot_motion();
  robot_quarter_turn_counterclockwise();
  test_align_hopper();
  pick_up_game_ball();
  /*robot_drive_vertic(2);
  robot_drive_horiz(3);
  robot_drive_vertic(3);
  robot_drive_horiz(1);
  robot_drive_vertic(4);
  robot_drive_horiz(3);*/
}

void test_place_gameball(){
  pre_board_scan();
}

void check_gameboard_sensors(){
  
}
