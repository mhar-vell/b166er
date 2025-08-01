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


//Upload this code to /ttyACM1.


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

#define HBRIDGE_3A        4         //Joint 3 motor pin A.
#define HBRIDGE_3B        9         //Joint 3 motor pin B.
#define HBRIDGE_4A        7         //Joint 4 motor pin A.
#define HBRIDGE_4B        8         //Joint 4 motor pin B.

#define PWM_3             6         //Joint 3 PWM.
#define PWM_4             5         //Joint 4 PWM.

#define ENABLE_3          A1        //Joint 3 enable.
#define ENABLE_4          A0        //Joint 4 enable.

#define MOTOR_3           3         //Definition for joint 3 motor manipulation.
#define MOTOR_4           4         //Definition for joint 4 motor manipulation.

#define ENCODER_3A        18        //Joint 3 encoder pin A.
#define ENCODER_3B        19        //Joint 3 encoder pin B.
#define ENCODER_4A        20        //Joint 4 encoder pin A.
#define ENCODER_4B        21        //Joint 4 encoder pin B.

#define LS_3A             34        //Joint 3 limit sensor pin A.
#define LS_3B             36        //Joint 3 limit sensor pin B.
#define LS_4A             38        //Joint 4 limit sensor pin A.
#define LS_4B             40        //Joint 4 limit sensor pin B.

#define RELAY             14        //Joint 3 brake relay pin.

#define RESET             1         //Flag to indicate the robot to go home and forget the last setpoints.
#define RETRY             2         //Flag to indicate the robot to go home and retry the last setpoints.

#define PWM_MAX_3_CW      50//130       //Max value for the joint 3 PWM when needs to go CW.
#define PWM_MIN_3_CW      0         //Min value for the joint 3 PWM when needs to go CW.

#define PWM_MAX_3_CCW     200//255       //Max value for the joint 3 PWM when needs to go CCW (against gravity).
#define PWM_MIN_3_CCW     100//180       //Min value for the joint 3 PWM when needs to go CCW (against gravity).

#define PWM_MAX_4         150//180       //Max value for the joint 4 PWM.
#define PWM_MIN_4         50        //Min value for the joint 4 PWM.

#define kP_3              0.030     //Controller proportional gain for joint 3.
#define kI_3              0.040     //Controller integral gain for joint 3.
#define kD_3              0.600     //Controller derivative gain for joint 3.

#define kP_4              0.030     //Controller proportional gain for joint 4.
#define kI_4              0.050     //Controller integral gain for joint 4.
#define kD_4              0.050     //Controller derivative gain for joint 4.

#define PWM_MAX_VAR       15        //Maximum variation allowed to the PWM between interactions.


/////////////////////////////////
///// VARIABLES DEFINITIONS /////
/////////////////////////////////

bool brake_flag =         HIGH;     //Flag to the brake status. HIGH = locked; LOW = released.
bool EMERGENCY_STOP =     false;    //Flag for the emergency stop, if set to TRUE.

bool LS_3Astat;                     //Joint 3 limit sensor status on pin A.
bool LS_3Bstat;                     //Joint 3 limit sensor status on pin B.
bool LS_4Astat;                     //Joint 4 limit sensor status on pin A.
bool LS_4Bstat;                     //Joint 4 limit sensor status on pin B.

bool RESET_3 =            false;    //Flag set to TRUE when joint 3 completed the RESET routine.
bool RESET_4 =            false;    //Flag set to TRUE when joint 4 completed the RESET routine.
bool already_reset =      false;    //Flag to avoid endless resets when ROS is not connected.
int GOHOME =              0;        //Controlls the reset behaviour when going to home position: 0 = nothing; 1 = RESET; 2 = RETRY.

long DEG2PUL_3 =          186;      //Pulses per degree on joint 3 encoder. Originaly was: 27958/110 = 254.164.
long DEG2PUL_4 =          154.56;   //Pulses per degree on joint 4 encoder. Originaly was: 17676/180 = 98.2.

long HOME_3 =             -123;   //Joint 3 degree value at home position.
long HOME_4 =             0;        //Joint 4 degree value at home position.

float control_loop_3 =    0;        //Timer for the controller loop on joint 3.
float control_loop_4 =    0;        //Timer for the controller loop on joint 4.
unsigned long wait_time = 0;        //Timer for waiting loop applications.
unsigned long timer_3 =   0;        //Auxiliar variable for time computations on joint 3.
unsigned long timer_4 =   0;        //Auxiliar variable for time computations on joint 4.

//Joint 3 controller variables.
float encoder_count_3 =   0;        //Encoder pulse count.
float setpoint_3 =        0;        //Setpoint, in pulse notation.
float last_setpoint_3 =   0;        //Last setpoint, in pulse notation.
float error_3 =           0;        //Error on pulse counting.
float acc_error_3 =       0;        //Accumulated error overtime.
float output_3 =          0;        //Controller PWM output.
float prev_output_3 =     0;        //Previous ontroller PWM output.
float tol_3 =             0;        //Error tolerance, in degree notation (its value is set on setup routine).
float last_error_3 =      0;        //Controller error in last loop.
float var_error_3 =       0;        //Variation on the error on each loop.
bool control_enable_3 =   true;     //Enable flag for the control actuation.
int PWM_MIN_3 =           150;      //Variable that changes with the direction of joint 3.
int PWM_MAX_3 =           150;      //Variable that changes with the direction of joint 3.

//Joint 4 controller variables.
float encoder_count_4 =   0;        //Encoder pulse count.
float setpoint_4 =        0;        //Setpoint, in pulse notation.
float last_setpoint_4 =   0;        //Last setpoint, in pulse notation.
float error_4 =           0;        //Error on pulse counting.
float acc_error_4 =       0;        //Accumulated error overtime.
float output_4 =          0;        //Controller PWM output.
float tol_4 =             0;        //Error tolerance, in degree notation (its value is set on setup routine).
float last_error_4 =      0;        //Controller error in last loop.
float var_error_4 =       0;        //Variation on the error on each loop.
bool control_enable_4 =   true;     //Enable flag for the control actuation.

char buff[16];                      //Auxiliar variable for log printing.


/////////////////////////////
///// ROS CONFIGURATION /////
/////////////////////////////

ros::NodeHandle nh;                                                     //ROS node handler.
movemaster_msg::status pub_msg_3;                                       //ROS message to be published with status of joint 3.
movemaster_msg::status pub_msg_4;                                       //ROS message to be published with status of joint 4.
ros::Publisher pub_3("/status_3", &pub_msg_3);                          //ROS publisher for the status of joint 3.
ros::Publisher pub_4("/status_4", &pub_msg_4);                          //ROS publisher for the status of joint 4.
void Callback(const movemaster_msg::setpoint &rec_msg);                 //ROS callback function for received setpoints. 
ros::Subscriber<movemaster_msg::setpoint> sub("/setpoints", &Callback); //ROS subscriber for received setpoints.


////////////////////////
///// SYSTEM SETUP /////
////////////////////////
void setup()
{
  //Limit sensor data pins are inputs.
  pinMode(LS_3A, INPUT);
  pinMode(LS_3B, INPUT);
  pinMode(LS_4A, INPUT);
  pinMode(LS_4B, INPUT);

  //Encoder data pins are inputs.
  pinMode(ENCODER_3A, INPUT);
  pinMode(ENCODER_3B, INPUT);
  pinMode(ENCODER_4A, INPUT);
  pinMode(ENCODER_4B, INPUT);

  //Break relay is an output. Starts locked.
  pinMode(RELAY, OUTPUT);
  brake_lock();

  //Enable pins are outputs.
  pinMode(ENABLE_3, OUTPUT);
  pinMode(ENABLE_4, OUTPUT);
  digitalWrite(ENABLE_3, HIGH);
  digitalWrite(ENABLE_4, HIGH);
  
  //H-bridge pins are outputs.
  pinMode(HBRIDGE_3A, OUTPUT);
  pinMode(HBRIDGE_3B, OUTPUT);
  pinMode(HBRIDGE_4A, OUTPUT);
  pinMode(HBRIDGE_4B, OUTPUT);

  //PWM pins are outputs.
  pinMode(PWM_3, OUTPUT);
  pinMode(PWM_4, OUTPUT);

  //Interruptions are attached to the encoder inputs.
  attachInterrupt(digitalPinToInterrupt(ENCODER_3A), CheckEncoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_4A), CheckEncoder4, RISING);

  //Tolerance for the controller, set to 1/4 of a degree.
  tol_3 = DEG2PUL_3/5;
  tol_4 = DEG2PUL_4/5;

  //Starts serial communication and ROS network.
  Serial.begin(9600);  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_3);
  nh.advertise(pub_4);

  //Starts the timers.
  timer_3 = millis();
  timer_4 = millis();
}


/////////////////////////////
///// MAIN CONTROL LOOP /////
/////////////////////////////
void loop()
{ 
  nh.spinOnce();

  //Executes the control loop only if ROS is conencted.
  if(nh.connected()){
    Publish3();
    Publish4();
  
    if (!already_reset){
      GOHOME = RESET;
      GoHome();
      already_reset = true;
    }
    
    //Only enters the control loop for joint 3 if it is enabled.
    if(control_enable_3){

        //Computes the joint 3 control variables.
        control_loop_3 = millis() - timer_3;
        timer_3 = millis();
        error_3 = encoder_count_3 - setpoint_3;
        var_error_3 = (error_3 - last_error_3) / (control_loop_3);
        acc_error_3 =+ error_3 * (control_loop_3);
    
        //Checks if joint 3 is out of tolerance.
        if (abs(error_3) > tol_3){
          pub_msg_3.IsDone = false;
          pub_3.publish(&pub_msg_3);

          //switches between the values for PWM limits.
          if (error_3 > 0){
            PWM_MIN_3 = PWM_MIN_3_CW;
            PWM_MAX_3 = PWM_MAX_3_CW;
          }else{
            PWM_MIN_3 = PWM_MIN_3_CCW;
            PWM_MAX_3 = PWM_MAX_3_CCW;
          }

          //Calculates the control output for joint 3.
          //output_3 = max(min(abs(kP_3*error_3 + kD_3*var_error_3),PWM_MAX),PWM_MIN_3);
          output_3 = max(min(abs(kP_3*error_3 + kI_3*acc_error_3 + kD_3*var_error_3),PWM_MAX_3),PWM_MIN_3);     

          //Prevents abrupt changes in PWM if direction is same.
          if(sign(error_3) == sign(last_error_3)){
            if (output_3 > prev_output_3){
              if ((output_3 - prev_output_3) > PWM_MAX_VAR){
                output_3 = max(min((prev_output_3 + PWM_MAX_VAR),PWM_MAX_3),PWM_MIN_3);
              }
            }else{
              if ((prev_output_3 - output_3) > PWM_MAX_VAR){
                output_3 = max(min((prev_output_3 - PWM_MAX_VAR),PWM_MAX_3),PWM_MIN_3);
              }
            }
          }else{ //if direction changes.
            output_3 = 0;
          }
          
          last_error_3 = error_3;
          prev_output_3 = output_3;

          if(brake_flag){
            brake_release();
          }

          //Checks which direction the motor of joint 3 must rotate. If output is positive, then direction is clockwise.
          if (error_3 > 0){
            if (!digitalRead(LS_3B)){
              motorGo(MOTOR_3, CW, output_3);
            }
            else{
              brake_lock();
              motorGo(MOTOR_3, STOP, 0);
            }
          }else{
            if (!digitalRead(LS_3A)){
              motorGo(MOTOR_3, CCW, output_3);
            }
            else{
              brake_lock();
              motorGo(MOTOR_3, STOP, 0);
            }
          }
        }else{
          //If the joint 3 error is in tolerance, stops ant sets the "job done" flag.
          brake_lock();
          motorGo(MOTOR_3, STOP, 0);
          output_3 = 0;
          acc_error_3 = 0;
          pub_msg_3.IsDone = true;
          pub_3.publish(&pub_msg_3);
        }
    }

    //Only enters the control loop for joint 4 if it is enabled.
    if(control_enable_4){

        //Computes the joint 4 control variables.
        control_loop_4 = millis() - timer_4;
        timer_4 = millis();
        error_4 = encoder_count_4 - setpoint_4;
        var_error_4 = (error_4 - last_error_4) / (control_loop_4);
        acc_error_4 =+ error_4 * (control_loop_4);
        last_error_4 = error_4;
    
        //Checks if joint 4 is out of tolerance.
        if (abs(error_4) > tol_4){
          pub_msg_4.IsDone = false;
          pub_4.publish(&pub_msg_4);
          
          //Calculates the control output for joint 4.
          //output_4 = max(min(abs(kP_4*error_4 + kD_4*var_error_4),PWM_MAX_4),PWM_MIN_4);         
          output_4 = max(min(abs(kP_4*error_4 + kI_4*acc_error_4 + kD_4*var_error_4),PWM_MAX_4),PWM_MIN_4);

          //Checks which direction the motor of joint 4 must rotate. If output is positive, then direction is clockwise.
          if (error_4 > 0){
            if (!digitalRead(LS_4B))
              motorGo(MOTOR_4, CW, output_4);
            else
              motorGo(MOTOR_4, STOP, 0);
          }else{
            if (!digitalRead(LS_4A))
              motorGo(MOTOR_4, CCW, output_4);
            else
              motorGo(MOTOR_4, STOP, 0);
          }
        }else{
          //If the joint 4 error is in tolerance, stops ant sets the "job done" flag.
          motorGo(MOTOR_4, STOP, 0);
          output_4 = 0;
          acc_error_4 = 0;
          pub_msg_4.IsDone = true;
          pub_4.publish(&pub_msg_4);
        }
    }
  }else{
      //If ROS is not connected, removes the reset flag and stops the motors.
      already_reset = false;
      acc_error_3 = 0;
      acc_error_4 = 0;
      motorGo(MOTOR_4, STOP, 0);
      brake_lock();
      motorGo(MOTOR_3, STOP, 0);
   }
}



/////////////////////
///// FUNCTIONS /////
/////////////////////

//Function  to publish information about joint 3.
void Publish3(){
  pub_msg_3.joint         =   "Joint 3";
  pub_msg_3.setpoint      =   setpoint_3;
  pub_msg_3.pulse_count   =   encoder_count_3/DEG2PUL_3;
  pub_msg_3.error         =   error_3;
  pub_msg_3.output        =   output_3;
  pub_msg_3.control_loop  =   control_loop_3;
  
  pub_3.publish(&pub_msg_3);
  nh.spinOnce();
}

//Function  to publish information about joint 4.
void Publish4(){
  pub_msg_4.joint         =   "Joint 4";
  pub_msg_4.setpoint      =   setpoint_4;
  pub_msg_4.pulse_count   =   encoder_count_4/DEG2PUL_4;
  pub_msg_4.error         =   error_4;
  pub_msg_4.output        =   output_4;
  pub_msg_4.control_loop  =   control_loop_4;
  
  pub_4.publish(&pub_msg_4);
  nh.spinOnce();
}

//Function to receive the setpoints for joints 1 and 2.
void Callback(const movemaster_msg::setpoint & rec_msg) {
  setpoint_3 = rec_msg.set_3 * DEG2PUL_3;
  setpoint_4 = rec_msg.set_4 * DEG2PUL_4;
  EMERGENCY_STOP = rec_msg.emergency_stop;
  GOHOME = rec_msg.GoHome;

  if(EMERGENCY_STOP){
    motorGo(MOTOR_3, STOP, 0);
    brake_lock();
    motorGo(MOTOR_4, STOP, 0);
  }
  
  if(GOHOME) GoHome();
}

//Function to reset te robot to home position.
//if GOHOME is 1, then the last setpoint is forgotten (RESET).
//if GOHOME is 2, then the last setpoint is repeated after the reset routine (RETRY).
void GoHome(){

  //Saves the last setpoints anyway.
  last_setpoint_3 = setpoint_3;
  last_setpoint_4 = setpoint_4;

  //First, adjusts joint 4 to CCW limit in order to avoid possible self-collisions.
  motorGo(MOTOR_4, CCW, PWM_MAX_4);
  LS_4Astat = digitalRead(LS_4A);
  while(!LS_4Astat){
      LS_4Astat = digitalRead(LS_4A);
      nh.spinOnce();
  }
  motorGo(MOTOR_4, STOP, 0);

  //Wait for 1 second.
  wait_time = millis();
  while(millis() < wait_time + 1000){
      motorGo(MOTOR_4, STOP, 0);
  }
  
  //joint 3 goes to home position, CW. Stops when LS activates.
  brake_release();
  motorGo(MOTOR_3, CW, PWM_MAX_3_CW);
  LS_3Bstat = digitalRead(LS_3B);
  while(!LS_3Bstat){
      LS_3Bstat = digitalRead(LS_3B);
      nh.spinOnce();
  }
  motorGo(MOTOR_3, STOP, 0);
  brake_lock();

  //Wait for 0.5 second.
  wait_time = millis();
  while(millis() < wait_time + 500){
      motorGo(MOTOR_4, STOP, 0);
  }

  //joint 4 goes to home position, CW. Stops at 0 degree mark.
  encoder_count_4 = 0;
  while(abs(encoder_count_4) < DEG2PUL_4*95){
    motorGo(MOTOR_4, CW, PWM_MAX_4);
    nh.spinOnce();
  }
  motorGo(MOTOR_4, STOP, 0);

  //Wait for 0.5 second.
  wait_time = millis();
  while(millis() < wait_time + 500){
      motorGo(MOTOR_4, STOP, 0);
  }

  //Resets the controller variables on joint 3.
  encoder_count_3 =    HOME_3*DEG2PUL_3;
  error_3 =            0;
  output_3 =           0;
  last_error_3 =       0;
  var_error_3 =        0;

  //Resets the controller variables on joint 4.
  encoder_count_4 =    HOME_4*DEG2PUL_4;
  error_4 =            0;
  output_4 =           0;
  last_error_4 =       0;
  var_error_4 =        0;

  //If it's a RETRY routine, try again the last setpoints. If it's a RESET routine, stays at home.
  if(GOHOME == RETRY){
    setpoint_3 = last_setpoint_3;
    setpoint_4 = last_setpoint_4;
  }else{
    setpoint_3 = encoder_count_3;
    setpoint_4 = encoder_count_4;
  }
}


//Function to activate brake on joint 4.
void brake_lock(){
  digitalWrite(RELAY,HIGH);
  brake_flag = HIGH;
}


//Function to release brake on joint 4.
void brake_release(){
  digitalWrite(RELAY,LOW);
  brake_flag = LOW;
}


//Interrupt function to count encoder pulses on joint 3.
void CheckEncoder3(){
  encoder_count_3 += digitalRead(ENCODER_3B) == HIGH ? -1 : +1;
}


//Interrupt function to count encoder pulses on joint 4.
void CheckEncoder4(){
  encoder_count_4 += digitalRead(ENCODER_4B) == HIGH ? -1 : +1;
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
    case MOTOR_3: {
        switch (dir)
        {
          case CW: {
              digitalWrite(HBRIDGE_3A, HIGH);
              digitalWrite(HBRIDGE_3B, LOW);
              break;
            }
          case CCW: {
              digitalWrite(HBRIDGE_3A, LOW);
              digitalWrite(HBRIDGE_3B, HIGH);
              break;
            }
          case STOP: {
              digitalWrite(HBRIDGE_3A, LOW);
              digitalWrite(HBRIDGE_3B, LOW);
              break;
            }
        }
        analogWrite(PWM_3, pwm);
        break;
      }

    case MOTOR_4: {
        switch (dir)
        {
          case CW: {
              digitalWrite(HBRIDGE_4A, HIGH);
              digitalWrite(HBRIDGE_4B, LOW);
              break;
            }
          case CCW: {
              digitalWrite(HBRIDGE_4A, LOW);
              digitalWrite(HBRIDGE_4B, HIGH);
              break;
            }
          case STOP: {
              digitalWrite(HBRIDGE_4A, LOW);
              digitalWrite(HBRIDGE_4B, LOW);
              break;
            }
        }
        analogWrite(PWM_4, pwm);
        break;
      }
  }
}
