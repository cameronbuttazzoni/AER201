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
#define HOP_DETECT_MAX_DIST 100 //Maximum distance values collected (in cm)
#define HOP_DETECT_MIN_DIST 5 //Minimum distance values polled (in cm), keep > 0
#define HOP_DETECT_PING_DELAY 30000 //time between pings from hopper detector distance sensor (min 30000) microseconds
#define HOPPER_DETECT_ERROR 3 // distance differences where program thinks it has detected change
#define HOPPER_DETECT_MID_LEG_DIFF 5 // minimum difference in distance for the mid leg compared to the outer legs
#define HOPPER_DETECT_MAX_ERROR 25 // max difference between leg values that trigger difference
#define HOPPER_DETECT_Y_ERROR 2 // distance that counts as the same value when comparing to y_robot
#define HOPPER_VALUES_IN_ROW 2 //Number of similar values in a row that need to be found
#define HOPPER_DETECT_INITIAL_X_LINE 6 //go to this x value (cm) for the initial hopper detection algorithm
#define HOP_DETECT_X_OFFSET 7 //distance from the distance sensor to the centre of the robot in gamefield x-direction (orientation is -PI/2)
#define HOP_DETECT_Y_OFFSET 13 //distance from the distance sensor to the centre of the robot in gamefield y-direction (orientation is -PI/2)
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
#define GAMEFIELD_RED_LINE_NUM 4 //the value of the center line in terms of which x-line it is

// WHEEL CONSTANTS
#define RIGHT_IR_PIN A0 //analog pin for the right wheel's IR sensor
#define WHEEL_IR_DELAY 5000 //time between checks for the wheel IR sensors MICROSECONDS
#define WHEEL_CIRCUMFERENCE 28.3 //circumference of the wheel (in cm)
#define WHEEL_NUM_HOLES 18 //number of holes in the wheel
#define POSITION_ERROR 2 //in cm must be within this distance for position checks MAKE SMALLER THEN MIN ENCODER CHANGE
#define IR_DIFFERENCE_VAL 10 //greater than or equal to this val counts as the sensors detecting, less than this is not detecting

// ROBOT DIMENSIONS
#define ROBOT_LENGTH_FRONT 12.5 //distance from point of rotation to front of robot
#define ROBOT_TURNING_RADIUS 10 //distance from the center of rotation to the wheels of the robot
#define DROP_BALL_X_OFFSET 4 //distance from the center of the robot to where the ball drops from in x direction when direct = -PI/2

// MOTOR CONSTANTS
#define RIGHT_WHEEL_ENABLE_PIN 2 //enable the right wheel motor
#define LEFT_WHEEL_ENABLE_PIN 3 //enable the left wheel motor
#define LEFT_FORWARD_PIN 24 //Set to high to move left wheel forward
#define LEFT_BACKWARD_PIN 22 //Set to high to move left wheel backward
#define RIGHT_FORWARD_PIN 26 //set to high to move right wheel forward
#define RIGHT_BACKWARD_PIN 28 //set to high to move right wheel backward
#define LEFT_WHEEL_MAX_SPEED 255 //max of 255
#define RIGHT_WHEEL_MAX_SPEED 255 //max of 255
#define LEFT_WHEEL_TURN_SPEED 130 //max of 255, set to 130
#define RIGHT_WHEEL_TURN_SPEED 130 //max of 255, set to 130
#define LEFT_WHEEL_GAMEBOARD_SPEED 155 //speed of left wheel when moving along gameboard, 155
#define RIGHT_WHEEL_GAMEBOARD_SPEED 155 //speed of right wheel when moving along gameboard, 155
#define LEFT_WHEEL_ALIGN_SPEED 95 //speed of left wheel when aligning to the line, set to 80
#define RIGHT_WHEEL_ALIGN_SPEED 95 //speed of left wheel when aligning to the line, set to 80
#define LEFT_WHEEL_ENCODER_SPEED 110 //speed of the left wheel when using the encoders during orienting on a hopper stage, 110
#define RIGHT_WHEEL_ENCODER_SPEED 110 //speed of the right wheel when using the encoders during orienting on a hopper stage, 110
#define LEFT_WHEEL_ORIENT_SPEED 200 //speed of the left wheel when aligning to a hopper, 90
#define RIGHT_WHEEL_ORIENT_SPEED 200 //speed of the right wheel when aligning to a hopper, 90
#define TURN_CORRECTION_DELAY 350 //after quarter turn reverse turn for this long to correct for overturning, 350

// LINE SENSORS
#define NUM_LINE_SENSORS 5 //number of line sensors
#define LEFT_LINE_SENSOR_PIN 45 //left line detecting black/white IR sensor pin
#define MID_LINE_SENSOR_PIN 47 //middle line detecting black/white IR sensor pin
#define RIGHT_LINE_SENSOR_PIN 49 //right line detecting black/white IR sensor pin
#define MAIN_LINE_SENSOR_PIN 51 //the line sensor keeping track of in between the wheels, at very center of robot
#define SIDE_LINE_SENSOR_PIN 43 //line sensor on RIGHT/LEFT side of robot
#define PULSE_LINE_SENSOR_TIME 3000 //send a pulse every "time to check all line sensors" + PULSE_LINE_SENSOR_TIME MICROSECONDS
#define GOOD_ON_TRACK_TIME 2000000 //microseconds that the robot should be on the line for
#define MIN_SPEED_CORRECTION_VALUE 10 //minimum value to change wheel speed by when veering off course
#define MAX_SPEED_CORRECTION_VALUE 30 //max value to change wheel speed for slight off course
#define ROBOT_ON_LINE_ERROR 30 //number of checks after line before we start checking if we passed line again

// BALL RELEASE AND PICKUP CONSTANTS
#define BALL_GRAB_FAN_PIN 38 //pin that controls the fan to suck up balls
#define BALL_GRAB_FAN_CONST_PIN 40 //always keep this HIGH
#define BALL_RELEASE_SERVO_PIN 50 //pin that controls the servo motor to release the ball
#define BALL_SUCTION_ON_TIME 7000 //amount of time the vacuum sucks for when picking up a ball MILLISECONDS
#define BALL_SUCTION_OFF_TIME 5000 //amount of time the robot delays for after turning off the fan MILLISECONDS
#define BALL_RELEASE_SERVO_INITIAL_VAL 136 //value for when the servo is closed, 136
#define BALL_RELEASE_SERVO_FINAL_VAL 102 //value for when the servo is open, 102
#define BALL_RELEASE_POSITION_ERROR 1.7 //in cm if the robot is within this distance of the column it will stop to place a ball
#define RELEASE_BALL_RESET_DELAY 2000 //delay time to reset the servo after placing a ball in MILLISECONDS, FOR TEST ONLY
#define RELEASE_BALL_Y_VALUE 160 //y_robot for when the robot is aligned to the game board
#define GAMEBOARD_INITIAL_X_ROBOT 6 //x line that the robot goes to when going to the gameboard
#define GAMEBOARD_INITIAL_Y_ROBOT 8 //y line that the robot goes to when going to the gameboard
#define BALL_GRAB_TRIG_PIN 35 //trig pin for ball grabbing ultrasonic sensor
#define BALL_GRAB_ECHO_PIN 37 //echo pin for ball grabbing ultrasonic sensor
#define BALL_GRAB_MAX_DIST 60 //max dist, 60
#define BALL_GRAB_PING_DELAY 30000 //delay for the front ultrasonic sensor
#define NUMBER_OF_HOP_ORIENT_DISTS 2 //must be even number, half this value is the number of values in a row that are averaged
#define BALL_GRAB_PROPER_DISTANCE 17 //distance the robot must be from the hopper to grab a ball
#define BALL_GRAB_DISTANCE_ERROR 1 //distance the robot must be within for the robot to try and grab the ball
#define ZERO_MEASUREMENTS_IN_ROW 2 //need this many 0s in between hopper legs
#define HOPPER_CENTER_LENGTH_X 13 //distance from leg to centre of hopper in x direction
#define HOPPER_CENTER_LENGTH_Y 7 //distance from leg to centre of hopper in y direction
#define HOP_ORIENT_SCAN_DIST_CORNER 41 //distance for the corner hoppers ESTIMATE

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
#define STOP_MOTION_DELAY 300 //everytime you stop motion, delay for this amount of time

// MATH
#define PI_OVER_TWO 1.5708
#define PI_THREE_OVER_TWO 4.7124
#define EPSILON 0.01 

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
const int line_sensor_order[] = {5, 1, 3, 4, 2}; //1 is left sensor, 2 is middle sensor, 3 is right sensor, 4 is main sensor, 5 is side sensor
const int line_sensor_delay[] = {180, 0, 0, 0, 50}; //cumulative delay, must be in order from smallest to largest, 220
const int ldr_wall_vals[] = {10, 10, 10, 10, 10, 10};
const int ldr_ball_vals[] = {10, 10, 10, 10, 10, 10};
int ldr_vals[] = {0, 0, 0, 0, 0, 0};
int line_sensor_checks = 0;
int left_line_sensor_val = HIGH; //last measured value of each line sensor
int mid_line_sensor_val = HIGH;
int right_line_sensor_val = HIGH;
int main_line_sensor_val = HIGH;
int side_line_sensor_val = HIGH;
NewPing hopper_detector(HOP_DETECT_TRIG_PIN, HOP_DETECT_ECHO_PIN, HOP_DETECT_MAX_DIST); // Create NewPing object for hopper detection
NewPing ball_grab_detector(BALL_GRAB_TRIG_PIN, BALL_GRAB_ECHO_PIN, BALL_GRAB_MAX_DIST); // Create NewPing object for hopper orientation
unsigned long ping_time = 0;     // holds time of the next ping
int ping_dist = 0;
unsigned long front_ping_time = 0;
float front_ping_dist = 0;
unsigned long wheel_ir_time = 0;     // holds time of the next check of wheel IR sensors
unsigned long line_sensor_time = 0; //holds time of the next check of the line sensors
int line_sensor_state = 0; //if 0 need to turn on line sensor reflect, if 1 need to check for colour
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
unsigned int next_ball_column = 0; //holds the column to play the next ball into
unsigned int next_hopper; //holds the next hopper to get a ball from
int left_wheel_speed = LEFT_WHEEL_MAX_SPEED; //speed of left wheel
int right_wheel_speed = RIGHT_WHEEL_MAX_SPEED; //speed of right wheel
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
void line_slightly_left_forward(); //robot is slightly to the left of the line
void line_slightly_right_forward(); //robot is slightly to the right of the line
void line_far_left_forward(); //robot is far left of the line
void line_far_right_forward(); //robot is far right of the line
void line_slightly_left_backward(); //robot is slightly to the left of the line
void line_slightly_right_backward(); //robot is slightly to the right of the line
void line_far_left_backward(); //robot is far left of the line
void line_far_right_backward(); //robot is far right of the line
int check_line_sensors_on_line(unsigned long cur_time); //return 1 if SIDE line sensor on line, else return 0
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
void align_robot(int prev_direct); //when robot is on line, align it to line
void update_line_sensors(); // updates the values of all of the line sensors
void send_hopper_detect_ping(unsigned long cur_time); //send ping and update ping_dist variable
void send_hopper_orient_ping(unsigned long cur_time);
int go_back_to_line(); //after getting a gameball, go back to the line
void go_back_to_line_error(int error_check); 
void orient_to_hopper(); //get right angle for grabbing ball from hopper
void align_to_hopper(); //get right distance for grabbing ball from hopper
void grab_ball_error(int error_check);

//Main Functions
void main_setup(); //main program setup code
void main_loop(); //main program loop code
//Task Functions - Place in setup function
void pick_up_game_ball();
void place_game_ball();
void navigate_hopper();
void move_around_obstacle();
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
  delay(2000);
  y_robot = 20;
  detect_hoppers();
}

void loop(){
  return;
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
  delay(300);
  unsigned long cur_time = micros();
  int check = 1;
  while (check != 0){ //need to get off of the initial line
    check = check_line_sensors_on_line(cur_time); 
    cur_time = micros();
  }
  while (check != 1){
    check = check_line_sensors_on_line(cur_time); //keep turning until side line sensor on line
    cur_time = micros();
  }
  stop_robot_motion();
  delay(STOP_MOTION_DELAY);
  robot_orient += PI_OVER_TWO;
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
  robot_orient -= PI_OVER_TWO;
  normalize_orient();
  start_robot_clockwise_align();
  delay(TURN_CORRECTION_DELAY);
  align_robot(1);
}

void robot_turn_up(){ //turns robot so that it is facing direction 0
  normalize_orient();
  if (abs(robot_orient - PI) < EPSILON || abs(robot_orient + PI) < EPSILON) robot_quarter_turn_clockwise(); //facing down
  if (abs(robot_orient + PI_OVER_TWO) < EPSILON){ //facing left
    robot_quarter_turn_clockwise();
  }
  if (abs(robot_orient - PI_OVER_TWO) < EPSILON){ //facing right
    robot_quarter_turn_counterclockwise();
  }
}

void robot_turn_right(){ //turns robot so that it is facing direction PI/2
  normalize_orient();
  if (abs(robot_orient + PI_OVER_TWO) < EPSILON) robot_quarter_turn_clockwise(); //facing left
  if (abs(robot_orient) < EPSILON){ //facing up
    robot_quarter_turn_clockwise();
  }
  if (abs(robot_orient - PI) < EPSILON || abs(robot_orient + PI) < EPSILON){ //facing down
    robot_quarter_turn_counterclockwise();
  }
}

void robot_turn_down(){ //turns robot so that it is facing direction PI or -PI
  normalize_orient();
  if (abs(robot_orient) < EPSILON) robot_quarter_turn_clockwise(); //facing up
  if (abs(robot_orient + PI_OVER_TWO) < EPSILON){ //facing left
    robot_quarter_turn_counterclockwise();
  }
  if (abs(robot_orient - PI_OVER_TWO) < EPSILON){ //facing right
    robot_quarter_turn_clockwise();
  }
}

void robot_turn_left(){ //turns robot so that it is facing direction -PI/2
  normalize_orient();
  if (abs(robot_orient - PI_OVER_TWO) < EPSILON) robot_quarter_turn_counterclockwise(); //robot facing right
  if (abs(robot_orient) < EPSILON){ //robot_facing up
    robot_quarter_turn_counterclockwise();
  }
  if (abs(robot_orient - PI) < EPSILON || abs(robot_orient + PI) < EPSILON){ //robot facing down
    robot_quarter_turn_clockwise();
  }
}

int check_slightly_right(unsigned long cur_time){ //return 1 if left sensor is HIGH
  if (cur_time >= line_sensor_time){
    update_line_sensor_vals(cur_time);
    if (line_sensor_checks == NUM_LINE_SENSORS){
      line_sensor_state = 0; //ready for new check
      if (left_line_sensor_val == HIGH && right_line_sensor_val == LOW) return 1;
      return 0;
    }
  }
  return 2; //didnt check
}

int check_slightly_left(unsigned long cur_time){ //return 1 if right sensor is HIGH
  if (cur_time >= line_sensor_time){
    update_line_sensor_vals(cur_time);
    if (line_sensor_checks == NUM_LINE_SENSORS){
      line_sensor_state = 0; //ready for new check
      if (right_line_sensor_val == HIGH && left_line_sensor_val == LOW) return 1;
      return 0;
    }
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
      if (prev_direct == 1){
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
  unsigned long cur_time;
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
  if (robot_orient >= 2 * PI - EPSILON || robot_orient <= EPSILON){ //robot is facing upwards
    y_line_robot += robot_direction;
    y_robot = y_line_robot * LINE_SEP_DIST;
  }
  if (abs(robot_orient - PI_OVER_TWO) <= EPSILON){ //robot is facing right
    x_line_robot += robot_direction;
    if (x_line_robot == GAMEFIELD_RED_LINE_NUM) x_line_robot += robot_direction;
    x_robot = x_line_robot * LINE_SEP_DIST;
  }
  if (abs(robot_orient - PI) <= EPSILON){ //robot is facing down
    y_line_robot -= robot_direction;
    y_robot = y_line_robot * LINE_SEP_DIST;
  }
  if (abs(robot_orient - PI_THREE_OVER_TWO) <= EPSILON){ //robot is facing left
    x_line_robot -= robot_direction;
    if (x_line_robot == GAMEFIELD_RED_LINE_NUM) x_line_robot -= robot_direction;
    x_robot = x_line_robot * LINE_SEP_DIST;
  }
}

void line_on_track_forward(unsigned long cur_time){ //back on track so drive straight again
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED;
}

void line_slightly_left_forward(){ //turn robot right
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED - MIN_SPEED_CORRECTION_VALUE;
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED;
}

void line_slightly_right_forward(){ //turn robot left
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED - MIN_SPEED_CORRECTION_VALUE;
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
}

void line_far_left_forward(){ //too far left so turn right
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED - MAX_SPEED_CORRECTION_VALUE;
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED;
}

void line_far_right_forward(){ //too far right so turn left
  left_wheel_speed = LEFT_WHEEL_MAX_SPEED - MAX_SPEED_CORRECTION_VALUE;
  right_wheel_speed = RIGHT_WHEEL_MAX_SPEED;
}

void line_on_track_backward(unsigned long cur_time){ //back on track so drive straight again
  line_on_track_forward(cur_time);
}

void line_slightly_left_backward(){
  line_slightly_right_forward(); //opposite direction since wheels spin opposite direction
}

void line_slightly_right_backward(){
  line_slightly_left_forward(); //opposite direction since wheels spin opposite direction
}

void line_far_left_backward(){ //too far left
  line_far_right_forward(); //opposite direction since wheels spin opposite direction
}

void line_far_right_backward(){
  line_far_left_forward(); //opposite direction since wheels spin opposite direction
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
  if (cur_time > wheel_ir_time){
    int right_ir = analogRead(RIGHT_IR_PIN);
    if (right_ir < IR_DIFFERENCE_VAL) right_ir = 0;
    if (right_ir >= IR_DIFFERENCE_VAL) right_ir = 1;
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
  }
}

int check_line_sensors_on_line(unsigned long cur_time){ //return 1 if SIDE line sensors on line, return 0 if not, return 2 if not updated yet
  if (cur_time >= line_sensor_time){
    update_line_sensor_vals(cur_time);
    if (line_sensor_checks == NUM_LINE_SENSORS){
      line_sensor_state = 0; //ready for new check
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
  ping_time = cur_time + HOP_DETECT_PING_DELAY; // add delay before another ping is sent
}

int detect_hoppers(){
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
      send_hopper_detect_ping(cur_time);
      Serial.println(ping_dist);
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
        if (ping_dist <prev_dist - HOPPER_DETECT_ERROR && ping_dist -(int)y_robot - HOP_DETECT_Y_OFFSET > HOPPER_DETECT_Y_ERROR && count == HOPPER_VALUES_IN_ROW) flag = 2; //found3rdleg
      }
      if (hoppers[cur_hopper-1].orient == 1){ // middle leg is closer
        if (ping_dist > prev_dist + HOPPER_DETECT_ERROR && count == HOPPER_VALUES_IN_ROW) flag = 2; // found the 3rd leg
      }
    }
    //if (flag == 2 && ping_dist - y_robot < HOPPER_DETECT_Y_ERROR && count == HOPPER_VALUES_IN_ROW){ // Past the hopper
      // WONT WORK SINCE DOESNT RETURN 0, SO NEW IF STATEMENT FIXES IT
    if (flag == 2 && abs(prev_dist - ping_dist) > HOPPER_DETECT_MID_LEG_DIFF && count == HOPPER_VALUES_IN_ROW){ // Past the hopper
      prev_dist = 0; //set prev_dist back to 0
      flag = 1; //ready to search for next hopper
      count = 0; //reset count to 0 since we have past the hopper
    }
    if (ping_dist > HOP_DETECT_MIN_DIST + (int) y_robot + HOP_DETECT_Y_OFFSET && flag == 1 && count == HOPPER_VALUES_IN_ROW){ // found a hopper
      if (prev_dist == 0){ // Found first leg of hopper
        prev_dist = ping_dist; // prev dist stores first leg of hopper distance
        prev_dist2 = ping_dist;
        count = 0;
        continue;
      }
      //found middle of hopper
      if (ping_dist > prev_dist + HOPPER_DETECT_MID_LEG_DIFF && ping_dist < prev_dist + HOPPER_DETECT_MAX_ERROR){ // Hopper is Orientation 0
      //if (ping_dist > prev_dist + HOPPER_DETECT_ERROR){ // Hopper is Orientation 0
        stop_robot_motion();
        delay(3000);
        hoppers[cur_hopper].x = ((int) x_robot + HOP_DETECT_X_OFFSET + HOP_DETECT_X_ERROR) / LINE_SEP_DIST;
        hoppers[cur_hopper].y = prev_dist / LINE_SEP_DIST;
        hoppers[cur_hopper].orient = 0;
        hoppers[cur_hopper].balls = RANDOM_HOPPER_BALLS_NUM;
        cur_hopper++; //start looking for the next hopper
        flag = 0;
        prev_dist = ping_dist; //update prev_dist to distance to the middle leg
        count = 0; //found middle leg of hopper so reset count to 0
      }
      if (ping_dist < prev_dist - HOPPER_DETECT_MID_LEG_DIFF && ping_dist > prev_dist - HOPPER_DETECT_MAX_ERROR){ // Hopper is Orientation 1
      //if (ping_dist < prev_dist - HOPPER_DETECT_ERROR){ // Hopper is Orientation 1
      stop_robot_motion();
        delay(3000);
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
  delay(STOP_MOTION_DELAY);
  return 0;
}


