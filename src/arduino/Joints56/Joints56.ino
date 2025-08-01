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


//Upload this code to /ttyACM0.


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
#define OPEN              3         //Definition for motorGo to open the grip.
#define CLOSE             4         //Definition for motorGo to close the grip.

#define HBRIDGE_5A        4         //Joint 5 motor pin A.
#define HBRIDGE_5B        9         //Joint 5 motor pin B.
#define HBRIDGE_6A        7         //Joint 6 motor pin A.
#define HBRIDGE_6B        8         //Joint 6 motor pin B.

#define PWM_5             6         //Joint 5 PWM.
#define PWM_6             5         //Joint 6 PWM.

#define ENABLE_5          A1        //Joint 5 enable.
#define ENABLE_6          A0        //Joint 6 enable.

#define MOTOR_5           3         //Definition for joint 5 motor manipulation.
#define MOTOR_6           4         //Definition for joint 3 motor manipulation.

#define ENCODER_5A        20        //Joint 5 encoder pin A.
#define ENCODER_5B        21        //Joint 5 encoder pin B.

#define LS_5A             34        //Joint 5 limit sensor pin A.
#define LS_5B             36        //Joint 5 limit sensor pin B.

#define RESET             1         //Flag to indicate the robot to go home and forget the last setpoints.
#define RETRY             2         //Flag to indicate the robot to go home and retry the last setpoints.

#define PWM_MAX           255       //Max value for the PWM (100% duty cicle).
#define PWM_MIN_5_CW      100       //Min value for the joint 5 PWM when needs to go CW.
#define PWM_MIN_5_CCW     255       //Min value for the joint 5 PWM when needs to go CCW (this joint has a mechanical problem in this direction).

#define kP_5              1.000     //Controller proportional gain for joint 5.
#define kD_5              0.000     //Controller derivative gain for joint 5.

/////////////////////////////////
///// VARIABLES DEFINITIONS /////
/////////////////////////////////

bool brake_flag =         HIGH;     //Flag to the brake status. HIGH = locked; LOW = released.
bool EMERGENCY_STOP =     false;    //Flag for the emergency stop, if set to TRUE.

bool LS_5Astat;                     //joint 5 limit sensor status on pin A.
bool LS_5Bstat;                     //joint 5 limit sensor status on pin B.

bool RESET_5 =            false;    //Flag set to TRUE when joint 5 completed the RESET routine.
bool RESET_6 =            false;    //Flag set to TRUE when joint 6 completed the RESET routine.
bool already_reset =      false;    //Flag to avoid endless resets when ROS is not connected.
int GOHOME =              0;        //Controlls the reset behaviour when going to home position: 0 = nothing; 1 = RESET; 2 = RETRY.

long DEG2PUL_5 =          116;    //Pulses per degree on joint 5 encoder. Originaly was: 20667/180 = 114.81.

long HOME_5 =             0;        //joint 5 degree value at home position.

float control_loop_5 =    0;        //Timer for the controller loop on joint 5.
unsigned long wait_time = 0;        //Timer for waiting loop applications.
unsigned long timer_5 =   0;        //Auxiliar variable for time computations on joint 5.

//joint 5 controller variables.
float encoder_count_5 =   0;        //Encoder pulse count.
float setpoint_5 =        0;        //Setpoint, in pulse notation.
float last_setpoint_5 =   0;        //Last setpoint, in pulse notation.
float error_5 =           0;        //Error on pulse counting.
float output_5 =          0;        //Controller PWM output.
float tol_5 =             0;        //Error tolerance, in degree notation (its value is set on setup routine).
float last_error_5 =      0;        //Controller error in last loop.
float var_error_5 =       0;        //Variation on the error on each loop.
bool control_enable_5 =   true;     //Enable flag for the control actuation.
int PWM_MIN_5 =           150;      //Variable that changes with the direction of joint 5.

bool aux_grip =           false;    //Auxiliar variable for the grip command.
bool last_aux_grip =      false;    //Auxiliar variable for the grip command.
int set_grip =            CLOSE;    //Sets the grip to OPEN or CLOSE.
int last_setpoint_6 =     CLOSE;    //Last setpoint for the grip.
float TimeGrip =          0;        //Time when the last command was sent to Grip.
float LastTimeGrip =      0;        //Time that passed since last command was sent to the Grip.

char buff[16];                      //Auxiliar variable for log printing.


/////////////////////////////
///// ROS CONFIGURATION /////
/////////////////////////////

ros::NodeHandle nh;                                                     //ROS node handler.
movemaster_msg::status pub_msg_5;                                       //ROS message to be published with status of joint 5.
movemaster_msg::status pub_msg_6;                                       //ROS message to be published with status of joint 6.
ros::Publisher pub_5("/status_5", &pub_msg_5);                          //ROS publisher for the status of joint 5.
ros::Publisher pub_6("/status_6", &pub_msg_6);                          //ROS publisher for the status of joint 6.
void Callback(const movemaster_msg::setpoint &rec_msg);                 //ROS callback function for received setpoints. 
ros::Subscriber<movemaster_msg::setpoint> sub("/setpoints", &Callback); //ROS subscriber for received setpoints.


////////////////////////
///// SYSTEM SETUP /////
////////////////////////
void setup()
{
  //Limit sensor data pins are inputs.
  pinMode(LS_5A, INPUT);
  pinMode(LS_5B, INPUT);
  
  //Encoder data pins are inputs.
  pinMode(ENCODER_5A, INPUT);
  pinMode(ENCODER_5B, INPUT);

  //Enable pins are outputs.
  pinMode(ENABLE_5, OUTPUT);
  pinMode(ENABLE_6, OUTPUT);
  digitalWrite(ENABLE_5, HIGH);
  digitalWrite(ENABLE_6, HIGH);
  
  //H-bridge pins are outputs.
  pinMode(HBRIDGE_5A, OUTPUT);
  pinMode(HBRIDGE_5B, OUTPUT);
  pinMode(HBRIDGE_6A, OUTPUT);
  pinMode(HBRIDGE_6B, OUTPUT);

  //PWM pins are outputs.
  pinMode(PWM_5, OUTPUT);
  pinMode(PWM_6, OUTPUT);

  //Interruptions are attached to the encoder inputs.
  attachInterrupt(digitalPinToInterrupt(ENCODER_5A), CheckEncoder5, RISING);
  
  //Tolerance for the controller, set to 1/4 of a degree.
  tol_5 = DEG2PUL_5/10;

  //Starts serial communication and ROS network.
  Serial.begin(9600);  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_5);
  nh.advertise(pub_6);

  //Starts the timers.
  timer_5 = millis();
}


/////////////////////////////
///// MAIN CONTROL LOOP /////
/////////////////////////////
void loop()
{ 
  nh.spinOnce();

  //Executes the control loop only if ROS is conencted.
  if(nh.connected()){
    Publish5();
    Publish6();
  
    if (!already_reset){
      GOHOME = RESET;
      GoHome();
      already_reset = true;
    }
    
    //Only enters the control loop for joint 5 if it is enabled.
    if(control_enable_5){

        //Computes the joint 5 control variables.
        control_loop_5 = millis() - timer_5;
        timer_5 = millis();
        error_5 = encoder_count_5 - setpoint_5;
        var_error_5 = (error_5 - last_error_5) / (control_loop_5/1000);
        last_error_5 = error_5;
    
        //Checks if joint 5 is out of tolerance.
        if (abs(error_5) > tol_5){
          pub_msg_5.IsDone = false;
          pub_5.publish(&pub_msg_5);

          //switches between the values for PWM_MIN.
          if (error_5 > 0){
            PWM_MIN_5 = PWM_MIN_5_CW;
          }else{
            PWM_MIN_5 = PWM_MIN_5_CCW;
          }   
          
          //Calculates the control output for joint 5.
          output_5 = max(min(abs(kP_5*error_5 + kD_5*var_error_5),PWM_MAX),PWM_MIN_5);         

          //Checks which direction the motor of joint 5 must rotate. If output is positive, then direction is clockwise.
          if (error_5 > 0){
            if (!digitalRead(LS_5B))
              motorGo(MOTOR_5, CW, output_5);
            else
              motorGo(MOTOR_5, STOP, 0);
          }else{
            if (!digitalRead(LS_5A))
              motorGo(MOTOR_5, CCW, output_5);
            else
              motorGo(MOTOR_5, STOP, 0);
          }
        }else{
          //If the joint 5 error is in tolerance, stops ant sets the "job done" flag.
          motorGo(MOTOR_5, STOP, 0);
          output_5 = 0;
          pub_msg_5.IsDone = true;
          pub_5.publish(&pub_msg_5);
        }
    }

    nh.spinOnce();

    //Sends command to the grip but only for 2 seconds.
    if((millis() - TimeGrip) > 2000)
      motorGo(MOTOR_6, STOP, 0);
    else
      motorGo(MOTOR_6, set_grip, 255);
    
  }else{
      //If ROS is not connected, removes the reset flag and stops the motors.
      already_reset = false;
      motorGo(MOTOR_5, STOP, 0);
      motorGo(MOTOR_6, STOP, 0);
   }
}



/////////////////////
///// FUNCTIONS /////
/////////////////////

//Function  to publish information about joint 5.
void Publish5(){
  pub_msg_5.joint         =   "Joint 5";
  pub_msg_5.setpoint      =   setpoint_5;
  pub_msg_5.pulse_count   =   encoder_count_5/DEG2PUL_5;
  pub_msg_5.error         =   error_5;
  pub_msg_5.output        =   output_5;
  pub_msg_5.control_loop  =   control_loop_5;
  
  pub_5.publish(&pub_msg_5);
  nh.spinOnce();
}

//Function  to publish information about the .
void Publish6(){
  pub_msg_6.joint         =   "Grip";
  pub_msg_6.setpoint      =   set_grip;
  pub_6.publish(&pub_msg_6);
  nh.spinOnce();
}

//Function to receive the setpoints for joints 1 and 2.
void Callback(const movemaster_msg::setpoint & rec_msg) {
  setpoint_5 = rec_msg.set_5 * DEG2PUL_5;
  aux_grip = rec_msg.set_GRIP;

  if (aux_grip != last_aux_grip){
    TimeGrip = millis();
    last_aux_grip = aux_grip;
  }

  if (aux_grip)
    set_grip = OPEN;
  else
    set_grip = CLOSE;
    
  EMERGENCY_STOP = rec_msg.emergency_stop;
  GOHOME = rec_msg.GoHome;

  if(EMERGENCY_STOP){
    motorGo(MOTOR_5, STOP, 0);
    motorGo(MOTOR_6, STOP, 0);
  }
  
  if(GOHOME) GoHome();
}

//Function to reset te robot to home position.
//if GOHOME is 1, then the last setpoint is forgotten (RESET).
//if GOHOME is 2, then the last setpoint is repeated after the reset routine (RETRY).
void GoHome(){

  //Saves the last setpoints anyway.
  last_setpoint_5 = setpoint_5;
  last_setpoint_6 = set_grip;

  //joint 5 goes to limit position, CCW. Stops when LS activates.
  motorGo(MOTOR_5, CCW, PWM_MAX);
  LS_5Astat = digitalRead(LS_5A);
  while(!LS_5Astat){
      LS_5Astat = digitalRead(LS_5A);
      nh.spinOnce();
  }
  motorGo(MOTOR_5, STOP, 0);

  //Wait for 0.5 seconds.
  wait_time = millis();
  while(millis() < wait_time + 500){
      motorGo(MOTOR_5, STOP, 0);
  }

  //After the LS activates, rotates CW until at home position (zero degree, resulting in a flat grip).
  encoder_count_5 = 0;
  while(abs(encoder_count_5) < DEG2PUL_5*167){
    motorGo(MOTOR_5, CW, PWM_MAX);
    nh.spinOnce();
  }
  motorGo(MOTOR_5, STOP, 0);

  //Wait for 0.5 seconds.
  wait_time = millis();
  while(millis() < wait_time + 500){
      motorGo(MOTOR_5, STOP, 0);
  }
  
  //Resets the controller variables on joint 5.
  encoder_count_5 =    HOME_5*DEG2PUL_5;
  error_5 =            0;
  output_5 =           0;
  last_error_5 =       0;
  var_error_5 =        0;

  //Closes the grip, activating the motor for 2 seconds.
  wait_time = millis();
  while(millis() < wait_time + 2000){
      motorGo(MOTOR_6, CLOSE, 255);
      nh.spinOnce();
  }
  motorGo(MOTOR_6, STOP, 0);
  

  //If it's a RETRY routine, try again the last setpoints. If it's a RESET routine, stays at home.
  if(GOHOME == RETRY){
    setpoint_5 = last_setpoint_5;
    set_grip = last_setpoint_6;
  }else{
    setpoint_5 = encoder_count_5;
    set_grip = CLOSE;
  }
}

//Interrupt function to count encoder pulses on joint 5.
void CheckEncoder5(){
  encoder_count_5 += digitalRead(ENCODER_5B) == HIGH ? -1 : +1;
}

//Function to control the direction and PWM of each motor.
void motorGo(int motor, int dir, int pwm)
{
  switch (motor)
  {
    case MOTOR_5: {
        switch (dir)
        {
          case CW: {
              digitalWrite(HBRIDGE_5A, HIGH);
              digitalWrite(HBRIDGE_5B, LOW);
              break;
            }
          case CCW: {
              digitalWrite(HBRIDGE_5A, LOW);
              digitalWrite(HBRIDGE_5B, HIGH);
              break;
            }
          case STOP: {
              digitalWrite(HBRIDGE_5A, LOW);
              digitalWrite(HBRIDGE_5B, LOW);
              break;
            }
        }
        analogWrite(PWM_5, pwm);
        break;
      }

    case MOTOR_6: {
        switch (dir)
        {
          case OPEN: {
              digitalWrite(HBRIDGE_6A, HIGH);
              digitalWrite(HBRIDGE_6B, LOW);
              break;
            }
          case CLOSE: {
              digitalWrite(HBRIDGE_6A, LOW);
              digitalWrite(HBRIDGE_6B, HIGH);
              break;
            }
          case STOP: {
              digitalWrite(HBRIDGE_6A, LOW);
              digitalWrite(HBRIDGE_6B, LOW);
              break;
            }
        }
        analogWrite(PWM_6, pwm);
        break;
      }
  }
}
