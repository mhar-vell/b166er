////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                        //
//    Developed by: Jonathan Cerbaro, Dieisson Martinelli                                 //
//    UTFPR - Federal University of Technology - Paraná                                   //
//    CPGEI - Graduate Program on Electrical Engineering and Applied Informatics          //
//    LASER - Advanced Laboratory of Embedded Systems and Robotics                        //
//    Advisor: Prof. Dr. André Schneider de Oliveira                                      //
//    Co-advisor: Prof. Dr. João Alberto Fabro                                            //
//    Last changed: 20/04/2022                                                            //
//                                                                                        //
////////////////////////////////////////////////////////////////////////////////////////////


//Upload this code to /ttyACM2.


/////////////////////
///// LIBRARIES /////
/////////////////////

#include <ros.h>
#include <movemaster_msg/setpoint.h>
#include <movemaster_msg/status.h>


//////////////////////////////////////////
///// PINS AND CONSTANTS DEFINITIONS /////
//////////////////////////////////////////

#define STOP              0         //Definition for motorGo stop function.
#define CW                1         //Definition for motorGo clockwise function.
#define CCW               2         //Definition for motorGo counter-clockwise function.

#define HBRIDGE_1A        4         //Joint 1 motor pin A.
#define HBRIDGE_1B        9         //Joint 1 motor pin B.
#define HBRIDGE_2A        8         //Joint 2 motor pin A.
#define HBRIDGE_2B        7         //Joint 2 motor pin B.

#define PWM_1             6         //Joint 1 PWM.
#define PWM_2A            3         //Joint 2 PWM A pin, since it's a BTS7960 H-bridge. Normal is on pin 5.
#define PWM_2B            2         //Joint 2 PWM B pin, since it's a BTS7960 H-bridge. Normal is on pin 5.

#define ENABLE_1          A1        //Joint 1 enable.
#define ENABLE_2A         10        //Joint 2 enable A pin, since it's a BTS7960 H-bridge. Normal is on pin A0.
#define ENABLE_2B         11        //Joint 2 enable B pin, since it's a BTS7960 H-bridge. Normal is on pin A0.

#define MOTOR_1           1         //Definition for joint 1 motor manipulation.
#define MOTOR_2           2         //Definition for joint 2 motor manipulation.

#define ENCODER_1A        18        //Joint 1 encoder pin A.
#define ENCODER_1B        19        //Joint 1 encoder pin B.
#define ENCODER_2A        20        //Joint 2 encoder pin A.
#define ENCODER_2B        21        //Joint 2 encoder pin B.

#define LS_1A             34        //Joint 1 limit sensor pin A.
#define LS_1B             36        //Joint 1 limit sensor pin B.
#define LS_2A             38        //Joint 2 limit sensor pin A.
#define LS_2B             40        //Joint 2 limit sensor pin B.

#define RELAY             14        //Joint 2 brake relay pin.

#define RESET             1         //Flag to indicate the robot to go home and forget the last setpoints.
#define RETRY             2         //Flag to indicate the robot to go home and retry the last setpoints.

#define PWM_MAX_1         100       //Max value for the joint 1 PWM.
#define PWM_MIN_1         50        //Min value for the joint 1 PWM.

#define PWM_MAX_2_CW      50        //Max value for the joint 2 PWM when needs to go CW.
#define PWM_MIN_2_CW      0         //Min value for the joint 2 PWM when needs to go CW.

#define PWM_MAX_2_CCW     150       //Max value for the joint 2 PWM when needs to go CCW (against gravity).
#define PWM_MIN_2_CCW     80        //Min value for the joint 2 PWM when needs to go CCW (against gravity).

#define kP_1              0.06      //Controller proportional gain for joint 1.
#define kD_1              0.30      //Controller derivative gain for joint 1.

#define kP_2              0.030     //Controller proportional gain for joint 2.
#define kI_2              0.020     //Controller integral gain for joint 2.
#define kD_2              0.500     //Controller derivative gain for joint 2.

#define PWM_MAX_VAR       15        //Maximum variation allowed to the PWM between interactions.

/////////////////////////////////
///// VARIABLES DEFINITIONS /////
/////////////////////////////////

bool brake_flag =         HIGH;     //Flag to the brake status. HIGH = locked; LOW = released.
bool EMERGENCY_STOP =     false;    //Flag for the emergency stop, if set to TRUE.

bool LS_1Astat =          0;        //Joint 1 limit sensor status on pin A.
bool LS_1Bstat =          0;        //Joint 1 limit sensor status on pin B.
bool LS_2Astat =          0;        //Joint 2 limit sensor status on pin A.
bool LS_2Bstat =          0;        //Joint 2 limit sensor status on pin B.

bool RESET_1 =            false;    //Flag set to TRUE when joint 1 completed the RESET routine.
bool RESET_2 =            false;    //Flag set to TRUE when joint 2 completed the RESET routine.
bool already_reset =      false;    //Flag to avoid endless resets when ROS is not connected.
int GOHOME =              0;        //Controlls the reset behaviour when going to home position: 0 = nothing; 1 = RESET; 2 = RETRY.

long DEG2PUL_1 =          185;      //Pulses per degree on joint 1 encoder. Originaly was: 67416/300 = 224.720.
long DEG2PUL_2 =          228;      //Pulses per degree on joint 2 encoder. Originaly was: 49311/130 = 379.315.

long HOME_1 =             0;        //Joint 1 degree value at home position.
long HOME_2 =             106;    //Joint 2 degree value at home position.

float control_loop_1 =    0;        //Timer for the controller loop on joint 1.
float control_loop_2 =    0;        //Timer for the controller loop on joint 2.
unsigned long wait_time = 0;        //Timer for waiting loop applications.
unsigned long timer_1 =   0;        //Auxiliar variable for time computations on joint 1.
unsigned long timer_2 =   0;        //Auxiliar variable for time computations on joint 2.

//Joint 1 controller variables.
float encoder_count_1 =   0;        //Encoder pulse count.
float setpoint_1 =        0;        //Setpoint, in pulse notation.
float last_setpoint_1 =   0;        //Last setpoint, in pulse notation.
float error_1 =           0;        //Error on pulse counting.
float output_1 =          0;        //Controller PWM output.
float prev_output_1 =     0;        //Previous ontroller PWM output.
float reset_output_1 =    0;        //PWM output during reset routine.
float tol_1 =             0;        //Error tolerance, in degree notation (its value is set on setup routine).
float last_error_1 =      0;        //Controller error in last loop.
float var_error_1 =       0;        //Variation on the error on each loop.
bool control_enable_1 =   true;     //Enable flag for the control actuation.

//Joint 2 controller variables.
float encoder_count_2 =   0;        //Encoder pulse count.
float setpoint_2 =        0;        //Setpoint, in pulse notation.
float last_setpoint_2 =   0;        //Last setpoint, in pulse notation.
float error_2 =           0;        //Error on pulse counting.
float acc_error_2 =       0;        //Accumulated error overtime.
float output_2 =          0;        //Controller PWM output.
float prev_output_2 =     0;        //Previous ontroller PWM output.
float reset_output_2 =    0;        //PWM output during reset routine.
float tol_2 =             0;        //Error tolerance, in degree notation (its value is set on setup routine).
float last_error_2 =      0;        //Controller error in last loop.
float var_error_2 =       0;        //Variation on the error on each loop.
bool control_enable_2 =   true;     //Enable flag for the control actuation.
int PWM_MIN_2 =           150;      //Variable that changes with the direction of joint 2.
int PWM_MAX_2 =           150;      //Variable that changes with the direction of joint 2.

long teste;
char buff[40];                      //Auxiliar variable for log printing.

//To use log on ROS, this is the format:
//sprintf(buff, "Testing: %d.", variable);
//nh.logwarn(buff);

/////////////////////////////
///// ROS CONFIGURATION /////
/////////////////////////////

ros::NodeHandle nh;                                                     //ROS node handler.
movemaster_msg::status pub_msg_1;                                       //ROS message to be published with status of joint 1.
movemaster_msg::status pub_msg_2;                                       //ROS message to be published with status of joint 2.
ros::Publisher pub_1("/status_1", &pub_msg_1);                          //ROS publisher for the status of joint 1.
ros::Publisher pub_2("/status_2", &pub_msg_2);                          //ROS publisher for the status of joint 2.
void Callback(const movemaster_msg::setpoint &rec_msg);                 //ROS callback function for received setpoints. 
ros::Subscriber<movemaster_msg::setpoint> sub("/setpoints", &Callback); //ROS subscriber for received setpoints.


////////////////////////
///// SYSTEM SETUP /////
////////////////////////
void setup()
{
  //Limit sensor data pins are inputs.
  pinMode(LS_1A, INPUT);
  pinMode(LS_1B, INPUT);
  pinMode(LS_2A, INPUT);
  pinMode(LS_2B, INPUT);

  //Encoder data pins are inputs.
  pinMode(ENCODER_1A, INPUT);
  pinMode(ENCODER_1B, INPUT);
  pinMode(ENCODER_2A, INPUT);
  pinMode(ENCODER_2B, INPUT);

  //Break relay is an output. Starts locked.
  pinMode(RELAY, OUTPUT);
  brake_lock();

  //Enable pins are outputs.
  pinMode(ENABLE_1, OUTPUT);
  pinMode(ENABLE_2A, OUTPUT);
  pinMode(ENABLE_2B, OUTPUT);
  digitalWrite(ENABLE_1, HIGH);
  digitalWrite(ENABLE_2A, HIGH);
  digitalWrite(ENABLE_2B, HIGH);
  
  //H-bridge pins are outputs.
  pinMode(HBRIDGE_1A, OUTPUT);
  pinMode(HBRIDGE_1B, OUTPUT);
  pinMode(HBRIDGE_2A, OUTPUT);
  pinMode(HBRIDGE_2B, OUTPUT);

  //PWM pins are outputs.
  pinMode(PWM_1, OUTPUT);
  pinMode(PWM_2A, OUTPUT);
  pinMode(PWM_2B, OUTPUT);

  //Interruptions are attached to the encoder inputs.
  attachInterrupt(digitalPinToInterrupt(ENCODER_1A), CheckEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2A), CheckEncoder2, RISING);

  //Tolerance for the controller, set to 1/4 of a degree.
  tol_1 = DEG2PUL_1/15;
  tol_2 = DEG2PUL_2/10;
  
  //Starts serial communication and ROS network.
  Serial.begin(9600);  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_1);
  nh.advertise(pub_2);

  //Starts the timers.
  timer_1 = millis();
  timer_2 = millis();
}


/////////////////////////////
///// MAIN CONTROL LOOP /////
/////////////////////////////
void loop()
{ 
  nh.spinOnce();
  
  //Executes the control loop only if ROS is conencted.
  if(nh.connected()){
    Publish1();
    Publish2();
  
    if (!already_reset){
      GOHOME = RESET;
      GoHome();
      already_reset = true;
    }
   
    //Only enters the control loop for joint 1 if it is enabled.
    if(control_enable_1){
        
        //Computes the joint 1 control variables.
        control_loop_1 = millis() - timer_1;
        timer_1 = millis();
        error_1 = encoder_count_1 - setpoint_1;
        var_error_1 = (error_1 - last_error_1) / (control_loop_1);
        
        //Checks if joint 1 is out of tolerance.
        if (abs(error_1) > tol_1){
          pub_msg_1.IsDone = false;
          pub_1.publish(&pub_msg_1);
          
          //Calculates the control output for joint 1.
          output_1 = max(min(abs(kP_1*error_1 + kD_1*var_error_1),PWM_MAX_1),PWM_MIN_1);
          
          /*if(sign(error_1) == sign(last_error_1)){
            if (output_1 > prev_output_1){
              if ((output_1 - prev_output_1) > PWM_MAX_VAR){
                output_1 = max(min((prev_output_1 + PWM_MAX_VAR),PWM_MAX_1),PWM_MIN_1);
              }
            }else{
              if ((prev_output_1 - output_1) > PWM_MAX_VAR){
                output_1 = max(min((prev_output_1 - PWM_MAX_VAR),PWM_MAX_1),PWM_MIN_1);
              }
            }
          }else{ //if direction changes
            output_1 = 0;
          }*/

          last_error_1 = error_1;
          prev_output_1 = output_1;
          
          //Checks which direction the motor of joint 1 must rotate. If error is positive, then direction is clockwise.
          if (error_1 > 0){
            if (!digitalRead(LS_1A))
              motorGo(MOTOR_1, CW, output_1);
            else
              motorGo(MOTOR_1, STOP, 0);
          }else{
            if (!digitalRead(LS_1B))
              motorGo(MOTOR_1, CCW, output_1);
            else
              motorGo(MOTOR_1, STOP, 0);
          }
        }else{
          //If the joint 1 error is in tolerance, stops ant sets the "job done" flag.
          motorGo(MOTOR_1, STOP, 0);
          output_1 = 0;
          pub_msg_1.IsDone = true;
          pub_1.publish(&pub_msg_1);
        }
    }

    //Only enters the control loop for joint 2 if it is enabled.
    if(control_enable_2){

        //Computes the joint 2 control variables.
        control_loop_2 = millis() - timer_2;
        timer_2 = millis();
        error_2 = encoder_count_2 - setpoint_2;
        var_error_2 = (error_2 - last_error_2) / (control_loop_2);
        acc_error_2 =+ error_2 * (control_loop_2);
    
        //Checks if joint 2 is out of tolerance.
        if (abs(error_2) > tol_2){
          pub_msg_2.IsDone = false;
          pub_2.publish(&pub_msg_2);

          //switches between the values for PWM limits.
          if (error_2 > 0){
            PWM_MIN_2 = PWM_MIN_2_CW;
            PWM_MAX_2 = PWM_MAX_2_CW;
          }else{
            PWM_MIN_2 = PWM_MIN_2_CCW;
            PWM_MAX_2 = PWM_MAX_2_CCW;
          }

          //Halves the PWM min cap if joint is above 80 degrees and going CCW, so it can go slower (not too much gravity influence).
          //if((encoder_count_2 > 80/DEG2PUL_2) && (error_2 < 0)){
          //  PWM_MIN_2 = 0.5*PWM_MIN_2;
          //}
          
          //Calculates the control output for joint 2.
          //output_2 = max(min(abs(kP_2*error_2 + kD_2*var_error_2),PWM_MAX_2),PWM_MIN_2);
          output_2 = max(min(abs(kP_2*error_2 + kI_2*acc_error_2 + kD_2*var_error_2),PWM_MAX_2),PWM_MIN_2);     
          
          //Prevents abrupt changes in PWM if direction is same.
          if(sign(error_2) == sign(last_error_2)){
            if (output_2 > prev_output_2){
              if ((output_2 - prev_output_2) > PWM_MAX_VAR){
                output_2 = max(min((prev_output_2 + PWM_MAX_VAR),PWM_MAX_2),PWM_MIN_2);
              }
            }else{
              if ((prev_output_2 - output_2) > PWM_MAX_VAR){
                output_2 = max(min((prev_output_2 - PWM_MAX_VAR),PWM_MAX_2),PWM_MIN_2);
              }
            }
          }else{ //if direction changes.
            output_2 = 0;
          }
          
          last_error_2 = error_2;
          prev_output_2 = output_2;

          if(brake_flag){
            brake_release();
          }

          //Checks which direction the motor of joint 2 must rotate. If error is positive, then direction is clockwise.
          if (error_2 > 0){
            if (!digitalRead(LS_2B)){
              motorGo(MOTOR_2, CW, output_2);
            }
            else{
              motorGo(MOTOR_2, STOP, 0);
              brake_lock();
            }
          }else{
            if (!digitalRead(LS_2A)){
              motorGo(MOTOR_2, CCW, output_2);
            }
            else{
              motorGo(MOTOR_2, STOP, 0);
              brake_lock();
            }
          }
          
        }else{
          //If the joint 2 error is in tolerance, stops ant sets the "job done" flag.
          motorGo(MOTOR_2, STOP, 0);
          brake_lock();
          output_2 = 0;
          acc_error_2 = 0;
          pub_msg_2.IsDone = true;
          pub_2.publish(&pub_msg_2);
        }
    }
  }else{
      //If ROS is not connected, removes the reset flag and stops the motors.
      already_reset = false;
      acc_error_2 = 0;
      motorGo(MOTOR_1, STOP, 0);
      motorGo(MOTOR_2, STOP, 0);
      brake_lock();
   }
}



/////////////////////
///// FUNCTIONS /////
/////////////////////

//Function  to publish information about joint 1.
void Publish1(){
  pub_msg_1.joint         =   "Joint 1";
  pub_msg_1.setpoint      =   setpoint_1;
  pub_msg_1.pulse_count   =   encoder_count_1/DEG2PUL_1;
  pub_msg_1.error         =   error_1;
  pub_msg_1.output        =   output_1;
  pub_msg_1.control_loop  =   control_loop_1;
  
  pub_1.publish(&pub_msg_1);
  nh.spinOnce();
}

//Function  to publish information about joint 2.
void Publish2(){
  pub_msg_2.joint         =   "Joint 2";
  pub_msg_2.setpoint      =   setpoint_2;
  pub_msg_2.pulse_count   =   encoder_count_2/DEG2PUL_2;
  pub_msg_2.error         =   error_2;
  pub_msg_2.output        =   output_2;
  pub_msg_2.control_loop  =   control_loop_2;
  
  pub_2.publish(&pub_msg_2);
  nh.spinOnce();
}

//Function to receive the setpoints for joints 1 and 2.
void Callback(const movemaster_msg::setpoint & rec_msg) {
  setpoint_1 = rec_msg.set_1 * DEG2PUL_1;
  setpoint_2 = rec_msg.set_2 * DEG2PUL_2;
  EMERGENCY_STOP = rec_msg.emergency_stop;
  GOHOME = rec_msg.GoHome;

  if(EMERGENCY_STOP){
    motorGo(MOTOR_1, STOP, 0);
    motorGo(MOTOR_2, STOP, 0);
    brake_lock();
  }
  
  if(GOHOME) GoHome();
}

//Function to reset te robot to home position.
//if GOHOME is 1, then the last setpoint is forgotten (RESET).
//if GOHOME is 2, then the last setpoint is repeated after the reset routine (RETRY).
void GoHome(){

  //Saves the last setpoints anyway.
  last_setpoint_1 = setpoint_1;
  last_setpoint_2 = setpoint_2;

  //joint 2 goes to home position, CCW. Stops when LS activates.
  reset_output_2 = 0;
  if(brake_flag){
    brake_release();
  }
  LS_2Astat = digitalRead(LS_2A);
  while(!LS_2Astat){
      //Forces small changes in PWM.
      if(reset_output_2 < 140)
        reset_output_2 = reset_output_2 + 1;
      else
        reset_output_2 = 140;

      delay(2);
      motorGo(MOTOR_2, CCW, reset_output_2);
      LS_2Astat = digitalRead(LS_2A);
      nh.spinOnce();
  }
  while(reset_output_2 > 0){
    reset_output_2 = reset_output_2 - 1;
    delay(2);
    motorGo(MOTOR_2, CCW, reset_output_2);
  }
  motorGo(MOTOR_2, STOP, 0);
  brake_lock();

  //Wait for 0.5 second.
  wait_time = millis();
  while(millis() < wait_time + 500){
      motorGo(MOTOR_2, STOP, 0);
  }
  
  //joint 1 goes to limit position, CCW. Stops when LS activates.
  reset_output_1 = 0;
  LS_1Bstat = digitalRead(LS_1B);
  while(!LS_1Bstat){
    //Forces small changes in PWM.
    if(reset_output_1 < PWM_MAX_1)
      reset_output_1 = reset_output_1 + 1;
    else
      reset_output_1 = PWM_MAX_1;

    delay(2);
    motorGo(MOTOR_1, CCW, reset_output_1);
    LS_1Bstat = digitalRead(LS_1B);
    nh.spinOnce();
  }
  while(reset_output_1 > 0){
    reset_output_1 = reset_output_1 - PWM_MAX_VAR;
    delay(2);
    motorGo(MOTOR_1, CCW, reset_output_1);
  }
  motorGo(MOTOR_1, STOP, 0);
  
  //Wait for 0.5 second.
  wait_time = millis();
  while(millis() < wait_time + 500){
      motorGo(MOTOR_1, STOP, 0);
  }
  
  //After the LS activates, rotates CW until at home position (half of the workspace of joint 1).
  encoder_count_1 = 0;
  
  while(encoder_count_1 > -DEG2PUL_1*104){
    //Forces small changes in PWM.
    if(reset_output_1 < PWM_MAX_1)
      reset_output_1 = reset_output_1 + 1;
    else
      reset_output_1 = PWM_MAX_1;

    //delay(2);
    //sprintf(buff, "Testing: %d.", encoder_count_1);
    //nh.logwarn(buff);

    motorGo(MOTOR_1, CW, reset_output_1);
    nh.spinOnce();
  }
  while(reset_output_1 > 0){
    reset_output_1 = reset_output_1 - PWM_MAX_VAR;
    delay(2);
    motorGo(MOTOR_1, CW, reset_output_1);
  }
  motorGo(MOTOR_1, STOP, 0);

  //Wait for 0.5 second.
  wait_time = millis();
  while(millis() < wait_time + 500){
      motorGo(MOTOR_1, STOP, 0);
  }
  
  //Resets the controller variables on joint 1.
  encoder_count_1 =    HOME_1*DEG2PUL_1;
  error_1 =            0;
  output_1 =           0;
  last_error_1 =       0;
  var_error_1 =        0;

  //Resets the controller variables on joint 2.
  encoder_count_2 =    HOME_2*DEG2PUL_2;
  error_2 =            0;
  output_2 =           0;
  last_error_2 =       0;
  var_error_2 =        0;

  //If it's a RETRY routine, try again the last setpoints. If it's a RESET routine, stays at home.
  if(GOHOME == RETRY){
    setpoint_1 = last_setpoint_1;
    setpoint_2 = last_setpoint_2;
  }else{
    setpoint_1 = encoder_count_1;
    setpoint_2 = encoder_count_2;
  }
}


//Function to activate brake on joint 2.
void brake_lock(){
  digitalWrite(RELAY,HIGH);
  brake_flag = HIGH;
}


//Function to release brake on joint 2.
void brake_release(){
  digitalWrite(RELAY,LOW);
  brake_flag = LOW;
}


//Interrupt function to count encoder pulses on joint 1.
void CheckEncoder1(){
  encoder_count_1 += digitalRead(ENCODER_1B) == HIGH ? -1 : +1;
}


//Interrupt function to count encoder pulses on joint 2.
void CheckEncoder2(){
  encoder_count_2 += digitalRead(ENCODER_2B) == HIGH ? +1 : -1;
  //encoder_count_2 += bitRead(PIND,ENCODER_2B) == HIGH ? +1 : -1;
}

static inline int8_t sign(int val){
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}


//Function to control the direction and PWM of each motor.
void motorGo(int motor, int dir, int pwm)
{
  
  switch (motor)
  {
    case MOTOR_1: {
        switch (dir)
        {
          case CW: {
              digitalWrite(HBRIDGE_1A, HIGH);
              digitalWrite(HBRIDGE_1B, LOW);
              break;
            }
          case CCW: {
              digitalWrite(HBRIDGE_1A, LOW);
              digitalWrite(HBRIDGE_1B, HIGH);
              break;
            }
          case STOP: {
              digitalWrite(HBRIDGE_1A, LOW);
              digitalWrite(HBRIDGE_1B, LOW);
              break;
            }
        }
        analogWrite(PWM_1, pwm);
        break;
      }

    case MOTOR_2: {
        switch (dir)
        {
          case CW: {
              analogWrite(PWM_2A,0);
              analogWrite(PWM_2B,pwm);
              break;
            }
          case CCW: {
              analogWrite(PWM_2A,pwm);
              analogWrite(PWM_2B,0);
              break;
            }
          case STOP: {
              analogWrite(PWM_2A,0);
              analogWrite(PWM_2B,0);
              break;
            }
        }
        break;
      }
  }
}
