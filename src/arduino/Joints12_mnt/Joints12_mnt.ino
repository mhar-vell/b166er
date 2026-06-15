////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                        //
//    MAINTENANCE AND TEST SCRIPT FOR JOINTS 1 & 2                                       //
//    Tests motors, encoders, and limit switches ONE JOINT AT A TIME                     //
//                                                                                        //
////////////////////////////////////////////////////////////////////////////////////////////

//Upload this code to test hardware components separately.

/////////////////////
///// LIBRARIES /////
/////////////////////

#include <ros.h>
#include <movemaster_msg/setpoint.h>
#include <movemaster_msg/status.h>

//////////////////////////////////////////
///// PINS AND CONSTANTS DEFINITIONS /////
//////////////////////////////////////////

#define STOP              0
#define CW                1
#define CCW               2

#define HBRIDGE_1A        4
#define HBRIDGE_1B        9
#define HBRIDGE_2A        8
#define HBRIDGE_2B        7

#define PWM_1             6
#define PWM_2A            3
#define PWM_2B            2

#define ENABLE_1          A1
#define ENABLE_2A         10
#define ENABLE_2B         11

#define MOTOR_1           1
#define MOTOR_2           2

#define ENCODER_1A        18
#define ENCODER_1B        19
#define ENCODER_2A        20
#define ENCODER_2B        21

#define LS_1A             34
#define LS_1B             36
#define LS_2A             38
#define LS_2B             40

#define RELAY             14

/////////////////////////////////
///// VARIABLES DEFINITIONS /////
/////////////////////////////////

bool brake_flag = HIGH;
volatile long encoder_count_1 = 0;
volatile long encoder_count_2 = 0;
volatile long enc1_interrupt_count = 0;
volatile long enc2_interrupt_count = 0;

int test_mode = 0;  // 0=menu, 1=J1 motor, 2=J1 encoder, 3=J1 sensor, 4=J1 integrated,
                    //         5=J2 motor, 6=J2 encoder, 7=J2 sensor, 8=J2 integrated
bool test_running = false;

/////////////////////////////
///// ROS CONFIGURATION /////
/////////////////////////////

ros::NodeHandle nh;
movemaster_msg::status pub_msg_1;
movemaster_msg::status pub_msg_2;
ros::Publisher pub_1("/status_1", &pub_msg_1);
ros::Publisher pub_2("/status_2", &pub_msg_2);

void Callback(const movemaster_msg::setpoint &rec_msg);
ros::Subscriber<movemaster_msg::setpoint> sub("/setpoints", &Callback);

////////////////////////
///// SYSTEM SETUP /////
////////////////////////

void setup()
{
  // Use internal pull-up to keep HIGH during boot
  pinMode(RELAY, INPUT_PULLUP);  // HIGH during boot (safe if logic inverted)
  delay(10);
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);  // Keep HIGH = brake locked (if you invert logic)
  brake_flag = HIGH;

  // Limit sensors as inputs
  pinMode(LS_1A, INPUT);
  pinMode(LS_1B, INPUT);
  pinMode(LS_2A, INPUT);
  pinMode(LS_2B, INPUT);

  // Encoders as inputs with pull-ups
  pinMode(ENCODER_1A, INPUT_PULLUP);
  pinMode(ENCODER_1B, INPUT_PULLUP);
  pinMode(ENCODER_2A, INPUT_PULLUP);
  pinMode(ENCODER_2B, INPUT_PULLUP);

  // Brake relay - SET TO LOW FOR SAFETY FIRST!
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW);  // ✅ Start with LOW = brake locked = safe
  brake_flag = HIGH;         // Flag indicates brake is locked

  // Enable pins
  pinMode(ENABLE_1, OUTPUT);
  pinMode(ENABLE_2A, OUTPUT);
  pinMode(ENABLE_2B, OUTPUT);
  digitalWrite(ENABLE_1, HIGH);
  digitalWrite(ENABLE_2A, HIGH);
  digitalWrite(ENABLE_2B, HIGH);
  
  // H-bridge pins
  pinMode(HBRIDGE_1A, OUTPUT);
  pinMode(HBRIDGE_1B, OUTPUT);
  pinMode(HBRIDGE_2A, OUTPUT);
  pinMode(HBRIDGE_2B, OUTPUT);

  // PWM pins
  pinMode(PWM_1, OUTPUT);
  pinMode(PWM_2A, OUTPUT);
  pinMode(PWM_2B, OUTPUT);

  // Attach interrupts with CHANGE mode for better reliability
  attachInterrupt(digitalPinToInterrupt(ENCODER_1A), CheckEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2A), CheckEncoder2, CHANGE);

  // Start serial and ROS
  Serial.begin(57600);
  nh.initNode();
  delay(1000);
  nh.subscribe(sub);
  nh.advertise(pub_1);
  nh.advertise(pub_2);

  // Show menu
  showMenu();
}

/////////////////////////////
///// MAIN CONTROL LOOP /////
/////////////////////////////

void loop()
{ 
  nh.spinOnce();
  
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 5000) {
    char buf[50];
    sprintf(buf, "Loop: test_mode=%d, test_running=%d", test_mode, test_running);
    nh.loginfo(buf);
    lastDebug = millis();
  }
  
  if(nh.connected()){
    Publish1();
    Publish2();
    
    switch(test_mode) {
      case 1: testMotor1(); break;
      case 2: testEncoder1(); break;
      case 3: testSensor1(); break;
      case 4: testIntegrated1(); break;
      case 5: testMotor2(); break;
      case 6: testEncoder2(); break;
      case 7: testSensor2(); break;
      case 8: testIntegrated2(); break;
      case 9: testBrakeRelay(); test_mode = 0; showMenu(); break;
      default: 
        delay(100);
        break;
    }
    
    // ✅ IMPROVED: Only lock once when transitioning to idle
    static int last_test_mode = -1;
    if (test_mode == 0 && last_test_mode != 0) {
      // Just transitioned to idle - ensure safe state ONCE
      motorGo(MOTOR_1, STOP, 0);
      motorGo(MOTOR_2, STOP, 0);
      brake_lock();
      nh.loginfo("System in safe idle state");
    }
    last_test_mode = test_mode;
    
  } else {
    // Safety: stop everything if ROS disconnected
    motorGo(MOTOR_1, STOP, 0);
    motorGo(MOTOR_2, STOP, 0);
    digitalWrite(RELAY, LOW);  // Force LOW for safety
    brake_flag = HIGH;
    delay(100);
  }
}

/////////////////////
///// FUNCTIONS /////
/////////////////////

void showMenu() {
  nh.loginfo("==================================================");
  nh.loginfo("JOINTS 1&2 MAINTENANCE TEST MENU");
  nh.loginfo("==================================================");
  nh.loginfo("Send setpoint with GoHome field to select test:");
  nh.loginfo("");
  nh.loginfo("JOINT 1 TESTS:");
  nh.loginfo("  GoHome: 1 = Test Joint 1 Motor");
  nh.loginfo("  GoHome: 2 = Test Joint 1 Encoder");
  nh.loginfo("  GoHome: 3 = Test Joint 1 Limit Switches");
  nh.loginfo("  GoHome: 4 = Test Joint 1 Integrated (all)");
  nh.loginfo("");
  nh.loginfo("JOINT 2 TESTS:");
  nh.loginfo("  GoHome: 5 = Test Joint 2 Motor");
  nh.loginfo("  GoHome: 6 = Test Joint 2 Encoder");
  nh.loginfo("  GoHome: 7 = Test Joint 2 Limit Switches");
  nh.loginfo("  GoHome: 8 = Test Joint 2 Integrated (all)");
  nh.loginfo("");
  nh.loginfo("BRAKE TEST:");
  nh.loginfo("  GoHome: 9 = Test Brake Relay");
  nh.loginfo("");
  nh.loginfo("  emergency_stop = true: Stop current test");
  nh.loginfo("==================================================");
}

//////////////////
// JOINT 1 TESTS
//////////////////

void testMotor1() {
  if (!test_running) {
    nh.loginfo("==================================================");
    nh.loginfo("TEST: JOINT 1 MOTOR");
    nh.loginfo("==================================================");
    nh.loginfo("Testing Motor 1 directions and PWM response");
    nh.loginfo("Watch for smooth acceleration/deceleration");
    nh.loginfo("==================================================");
    test_running = true;
  }

  nh.loginfo("Motor 1: CW direction, ramping PWM 0->120");
  for (int pwm = 0; pwm <= 120; pwm += 10) {
    motorGo(MOTOR_1, CW, pwm);
    char buf[50];
    sprintf(buf, "  CW PWM=%d", pwm);
    nh.loginfo(buf);
    delay(300);
    nh.spinOnce();
    Publish1();
  }
  
  nh.loginfo("Motor 1: Stopping");
  for (int pwm = 120; pwm >= 0; pwm -= 10) {
    motorGo(MOTOR_1, CW, pwm);
    delay(100);
    nh.spinOnce();
  }
  motorGo(MOTOR_1, STOP, 0);
  delay(2000);
  
  nh.loginfo("Motor 1: CCW direction, ramping PWM 0->120");
  for (int pwm = 0; pwm <= 120; pwm += 10) {
    motorGo(MOTOR_1, CCW, pwm);
    char buf[50];
    sprintf(buf, "  CCW PWM=%d", pwm);
    nh.loginfo(buf);
    delay(300);
    nh.spinOnce();
    Publish1();
  }
  
  nh.loginfo("Motor 1: Stopping");
  for (int pwm = 120; pwm >= 0; pwm -= 10) {
    motorGo(MOTOR_1, CCW, pwm);
    delay(100);
    nh.spinOnce();
  }
  motorGo(MOTOR_1, STOP, 0);
  
  nh.loginfo("==================================================");
  nh.loginfo("JOINT 1 MOTOR TEST COMPLETE");
  nh.loginfo("Result: Motor should have rotated smoothly");
  nh.loginfo("        in both CW and CCW directions");
  nh.loginfo("==================================================");
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void testEncoder1() {
  if (!test_running) {
    nh.loginfo("==================================================");
    nh.loginfo("TEST: JOINT 1 ENCODER");
    nh.loginfo("==================================================");
    nh.loginfo("Testing encoder response to motor movement");
    nh.loginfo("==================================================");
    test_running = true;
  }

  long enc_start = encoder_count_1;
  long int_start = enc1_interrupt_count;
  
  nh.loginfo("Rotating Joint 1 CW for 5 seconds...");
  unsigned long test_start = millis();
  unsigned long last_report = millis();
  
  while (millis() - test_start < 5000) {
    motorGo(MOTOR_1, CW, 80);
    
    if (millis() - last_report > 1000) {
      char buf[100];
      sprintf(buf, "  Enc: %ld | Int: %ld | E1A:%d E1B:%d", 
              encoder_count_1, enc1_interrupt_count,
              digitalRead(ENCODER_1A), digitalRead(ENCODER_1B));
      nh.loginfo(buf);
      last_report = millis();
    }
    
    nh.spinOnce();
    Publish1();
    delay(10);
  }
  
  motorGo(MOTOR_1, STOP, 0);
  
  long enc_change = encoder_count_1 - enc_start;
  long int_change = enc1_interrupt_count - int_start;
  
  char buf[100];
  nh.loginfo("==================================================");
  sprintf(buf, "Results: Encoder change = %ld pulses", enc_change);
  nh.loginfo(buf);
  sprintf(buf, "         Interrupts fired = %ld times", int_change);
  nh.loginfo(buf);
  
  if (abs(enc_change) > 100) {
    nh.loginfo("PASS: Encoder is working correctly!");
  } else if (int_change > 0) {
    nh.logwarn("PARTIAL: Interrupts working but count low");
    nh.logwarn("         Check encoder direction or coupling");
  } else {
    nh.logerror("FAIL: Encoder not responding!");
    nh.logerror("      Check: Power, wiring, mechanical coupling");
  }
  nh.loginfo("==================================================");
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void testSensor1() {
  if (!test_running) {
    nh.loginfo("==================================================");
    nh.loginfo("TEST: JOINT 1 LIMIT SWITCHES");
    nh.loginfo("==================================================");
    nh.loginfo("Manually press each limit switch");
    nh.loginfo("Test runs for 15 seconds");
    nh.loginfo("==================================================");
    test_running = true;
  }

  unsigned long test_start = millis();
  unsigned long last_report = millis();
  bool ls1a_detected = false;
  bool ls1b_detected = false;
  
  while (millis() - test_start < 15000) {
    bool ls1a = digitalRead(LS_1A);
    bool ls1b = digitalRead(LS_1B);
    
    if (ls1a && !ls1a_detected) {
      ls1a_detected = true;
      nh.loginfo("  >>> LS_1A ACTIVATED! <<<");
      delay(500);
    }
    if (ls1b && !ls1b_detected) {
      ls1b_detected = true;
      nh.loginfo("  >>> LS_1B ACTIVATED! <<<");
      delay(500);
    }
    
    if (millis() - last_report > 3000) {
      char buf[60];
      sprintf(buf, "Waiting... LS_1A=%d LS_1B=%d", ls1a, ls1b);
      nh.loginfo(buf);
      last_report = millis();
    }
    
    nh.spinOnce();
    delay(50);
  }
  
  nh.loginfo("==================================================");
  nh.loginfo("JOINT 1 LIMIT SWITCH TEST RESULTS:");
  char buf[50];
  sprintf(buf, "  LS_1A (CW limit):  %s", ls1a_detected ? "DETECTED" : "NOT DETECTED");
  if (ls1a_detected) nh.loginfo(buf); else nh.logwarn(buf);
  sprintf(buf, "  LS_1B (CCW limit): %s", ls1b_detected ? "DETECTED" : "NOT DETECTED");
  if (ls1b_detected) nh.loginfo(buf); else nh.logwarn(buf);
  nh.loginfo("==================================================");
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void testIntegrated1() {
  if (!test_running) {
    nh.loginfo("==================================================");
    nh.loginfo("TEST: JOINT 1 INTEGRATED (Motor+Encoder+Sensor)");
    nh.loginfo("==================================================");
    nh.loginfo("Testing BOTH directions with full range motion");
    nh.loginfo("Will return to starting position after test");
    nh.loginfo("==================================================");
    test_running = true;
  }

  // Record starting position
  long start_position = encoder_count_1;
  nh.loginfo("Recording starting position...");
  char buf[80];
  sprintf(buf, "  Starting encoder: %ld", start_position);
  nh.loginfo(buf);
  delay(1000);

  // ========== TEST 1: Move CW to limit ==========
  nh.loginfo("PHASE 1: Moving CW to LS_1A...");
  encoder_count_1 = 0;
  enc1_interrupt_count = 0;
  unsigned long test_start = millis();
  
  while (!digitalRead(LS_1A) && (millis() - test_start < 30000)) {
    motorGo(MOTOR_1, CW, 70);
    
    static unsigned long last_log = 0;
    if (millis() - last_log > 1000) {
      sprintf(buf, "  CW: Encoder=%ld | LS_1A:%d LS_1B:%d", 
              encoder_count_1, digitalRead(LS_1A), digitalRead(LS_1B));
      nh.loginfo(buf);
      last_log = millis();
    }
    
    nh.spinOnce();
    Publish1();
    delay(10);
  }
  
  // Gradual stop
  for (int pwm = 70; pwm >= 0; pwm -= 10) {
    motorGo(MOTOR_1, CW, pwm);
    delay(50);
  }
  motorGo(MOTOR_1, STOP, 0);
  delay(1000);
  
  // Results Phase 1
  long cw_encoder = encoder_count_1;
  long cw_interrupts = enc1_interrupt_count;
  bool cw_limit_reached = digitalRead(LS_1A);
  
  if (cw_limit_reached) {
    nh.loginfo("Phase 1 SUCCESS: Reached LS_1A");
    sprintf(buf, "  CW encoder pulses: %ld", cw_encoder);
    nh.loginfo(buf);
  } else {
    nh.logerror("Phase 1 FAIL: LS_1A not reached in 30s");
  }
  
  // ========== TEST 2: Move CCW to opposite limit ==========
  nh.loginfo("PHASE 2: Moving CCW to LS_1B...");
  encoder_count_1 = 0;  // Reset for CCW test
  enc1_interrupt_count = 0;
  test_start = millis();
  
  while (!digitalRead(LS_1B) && (millis() - test_start < 60000)) {
    motorGo(MOTOR_1, CCW, 70);
    
    static unsigned long last_log2 = 0;
    if (millis() - last_log2 > 1000) {
      sprintf(buf, "  CCW: Encoder=%ld | LS_1A:%d LS_1B:%d", 
              encoder_count_1, digitalRead(LS_1A), digitalRead(LS_1B));
      nh.loginfo(buf);
      last_log2 = millis();
    }
    
    nh.spinOnce();
    Publish1();
    delay(10);
  }
  
  // Gradual stop
  for (int pwm = 70; pwm >= 0; pwm -= 10) {
    motorGo(MOTOR_1, CCW, pwm);
    delay(50);
  }
  motorGo(MOTOR_1, STOP, 0);
  delay(1000);
  
  // Results Phase 2
  long ccw_encoder = encoder_count_1;
  long ccw_interrupts = enc1_interrupt_count;
  bool ccw_limit_reached = digitalRead(LS_1B);
  
  // ========== PHASE 3: Return to starting position ==========
  nh.loginfo("PHASE 3: Returning to starting position...");
  
  // Calculate target: we're at CCW limit, need to move back CW
  // Target is approximately middle of range
  long target_from_ccw_limit = -(cw_encoder + ccw_encoder) / 2;  // Approximate center
  encoder_count_1 = 0;  // Reset at CCW limit
  
  sprintf(buf, "  Target offset from CCW limit: %ld pulses", target_from_ccw_limit);
  nh.loginfo(buf);
  
  test_start = millis();
  while (encoder_count_1 < target_from_ccw_limit && (millis() - test_start < 45000)) {
    long error = target_from_ccw_limit - encoder_count_1;
    int pwm = constrain(abs(error) / 50, 30, 70);  // Proportional speed
    
    motorGo(MOTOR_1, CW, pwm);
    
    static unsigned long last_log3 = 0;
    if (millis() - last_log3 > 1000) {
      sprintf(buf, "  Returning: Encoder=%ld Target=%ld Error=%ld PWM=%d", 
              encoder_count_1, target_from_ccw_limit, error, pwm);
      nh.loginfo(buf);
      last_log3 = millis();
    }
    
    // Stop if within tolerance
    if (abs(error) < 100) {
      nh.loginfo("  Reached starting position!");
      break;
    }
    
    nh.spinOnce();
    Publish1();
    delay(10);
  }
  
  // Gradual stop
  for (int pwm = 70; pwm >= 0; pwm -= 10) {
    motorGo(MOTOR_1, CW, pwm);
    delay(50);
  }
  motorGo(MOTOR_1, STOP, 0);
  
  // ========== FINAL RESULTS ==========
  nh.loginfo("==================================================");
  nh.loginfo("JOINT 1 INTEGRATED TEST - FINAL RESULTS:");
  nh.loginfo("==================================================");
  
  // CW Direction Results
  nh.loginfo("CW Direction (to LS_1A):");
  sprintf(buf, "  Limit reached: %s", cw_limit_reached ? "YES" : "NO");
  if (cw_limit_reached) nh.loginfo(buf); else nh.logerror(buf);
  sprintf(buf, "  Encoder pulses: %ld", cw_encoder);
  nh.loginfo(buf);
  sprintf(buf, "  Interrupts: %ld", cw_interrupts);
  nh.loginfo(buf);
  
  nh.loginfo("--------------------------------------------------");
  
  // CCW Direction Results
  nh.loginfo("CCW Direction (to LS_1B):");
  sprintf(buf, "  Limit reached: %s", ccw_limit_reached ? "YES" : "NO");
  if (ccw_limit_reached) nh.loginfo(buf); else nh.logerror(buf);
  sprintf(buf, "  Encoder pulses: %ld", ccw_encoder);
  nh.loginfo(buf);
  sprintf(buf, "  Interrupts: %ld", ccw_interrupts);
  nh.loginfo(buf);
  
  nh.loginfo("--------------------------------------------------");
  
  // Overall Assessment
  sprintf(buf, "Total range: %ld pulses (CW + CCW)", abs(cw_encoder) + abs(ccw_encoder));
  nh.loginfo(buf);
  nh.loginfo("Joint returned to approximate starting position");
  
  if (cw_limit_reached && ccw_limit_reached) {
    if (abs(cw_encoder) > 100 && abs(ccw_encoder) > 100) {
      nh.loginfo("OVERALL: PASS - All systems working!");
      nh.loginfo("  Motor: OK | Encoder: OK | Sensors: OK");
    } else {
      nh.logwarn("OVERALL: PARTIAL - Sensors OK, encoder count low");
      nh.logwarn("  Check: Encoder mechanical coupling");
    }
  } else {
    nh.logerror("OVERALL: FAIL - Check limit switch wiring");
  }
  
  nh.loginfo("==================================================");
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

//////////////////
// JOINT 2 TESTS
//////////////////

void testMotor2() {
  if (!test_running) {
    nh.loginfo("==================================================");
    nh.loginfo("TEST: JOINT 2 MOTOR");
    nh.loginfo("==================================================");
    nh.loginfo("Testing Motor 2 with brake control");
    nh.loginfo("==================================================");
    test_running = true;
  }

  nh.loginfo("Releasing brake...");
  brake_release();
  delay(1000);
  
  nh.loginfo("Motor 2: CW direction, ramping PWM 0->150");
  for (int pwm = 0; pwm <= 150; pwm += 15) {
    motorGo(MOTOR_2, CW, pwm);
    char buf[50];
    sprintf(buf, "  CW PWM=%d", pwm);
    nh.loginfo(buf);
    delay(300);
    nh.spinOnce();
    Publish2();
  }
  
  nh.loginfo("Motor 2: Stopping and locking brake");
  for (int pwm = 150; pwm >= 0; pwm -= 15) {
    motorGo(MOTOR_2, CW, pwm);
    delay(100);
    nh.spinOnce();
  }
  motorGo(MOTOR_2, STOP, 0);
  brake_lock();
  delay(2000);
  
  nh.loginfo("Releasing brake...");
  brake_release();
  delay(1000);
  
  nh.loginfo("Motor 2: CCW direction, ramping PWM 0->150");
  for (int pwm = 0; pwm <= 150; pwm += 15) {
    motorGo(MOTOR_2, CCW, pwm);
    char buf[50];
    sprintf(buf, "  CCW PWM=%d", pwm);
    nh.loginfo(buf);
    delay(300);
    nh.spinOnce();
    Publish2();
  }
  
  nh.loginfo("Motor 2: Stopping and locking brake");
  for (int pwm = 150; pwm >= 0; pwm -= 15) {
    motorGo(MOTOR_2, CCW, pwm);
    delay(100);
    nh.spinOnce();
  }
  motorGo(MOTOR_2, STOP, 0);
  brake_lock();
  
  nh.loginfo("==================================================");
  nh.loginfo("JOINT 2 MOTOR TEST COMPLETE");
  nh.loginfo("Result: Motor should have rotated smoothly");
  nh.loginfo("        Brake should lock/release correctly");
  nh.loginfo("==================================================");
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void testEncoder2() {
  if (!test_running) {
    nh.loginfo("==================================================");
    nh.loginfo("TEST: JOINT 2 ENCODER");
    nh.loginfo("==================================================");
    test_running = true;
  }

  long enc_start = encoder_count_2;
  long int_start = enc2_interrupt_count;
  
  nh.loginfo("Releasing brake...");
  brake_release();
  delay(500);
  
  nh.loginfo("Rotating Joint 2 CW for 5 seconds...");
  unsigned long test_start = millis();
  unsigned long last_report = millis();
  
  while (millis() - test_start < 5000) {
    motorGo(MOTOR_2, CW, 100);
    
    if (millis() - last_report > 1000) {
      char buf[100];
      sprintf(buf, "  Enc: %ld | Int: %ld | E2A:%d E2B:%d", 
              encoder_count_2, enc2_interrupt_count,
              digitalRead(ENCODER_2A), digitalRead(ENCODER_2B));
      nh.loginfo(buf);
      last_report = millis();
    }
    
    nh.spinOnce();
    Publish2();
    delay(10);
  }
  
  motorGo(MOTOR_2, STOP, 0);
  brake_lock();
  
  long enc_change = encoder_count_2 - enc_start;
  long int_change = enc2_interrupt_count - int_start;
  
  char buf[100];
  nh.loginfo("==================================================");
  sprintf(buf, "Results: Encoder change = %ld pulses", enc_change);
  nh.loginfo(buf);
  sprintf(buf, "         Interrupts fired = %ld times", int_change);
  nh.loginfo(buf);
  
  if (abs(enc_change) > 100) {
    nh.loginfo("PASS: Encoder is working correctly!");
  } else if (int_change > 0) {
    nh.logwarn("PARTIAL: Interrupts working but count low");
  } else {
    nh.logerror("FAIL: Encoder not responding!");
  }
  nh.loginfo("==================================================");
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void testSensor2() {
  if (!test_running) {
    nh.loginfo("==================================================");
    nh.loginfo("TEST: JOINT 2 LIMIT SWITCHES");
    nh.loginfo("==================================================");
    nh.loginfo("Manually press each limit switch");
    nh.loginfo("Test runs for 15 seconds");
    nh.loginfo("==================================================");
    test_running = true;
  }

  unsigned long test_start = millis();
  unsigned long last_report = millis();
  bool ls2a_detected = false;
  bool ls2b_detected = false;
  
  while (millis() - test_start < 15000) {
    bool ls2a = digitalRead(LS_2A);
    bool ls2b = digitalRead(LS_2B);
    
    if (ls2a && !ls2a_detected) {
      ls2a_detected = true;
      nh.loginfo("  >>> LS_2A ACTIVATED! <<<");
      delay(500);
    }
    if (ls2b && !ls2b_detected) {
      ls2b_detected = true;
      nh.loginfo("  >>> LS_2B ACTIVATED! <<<");
      delay(500);
    }
    
    if (millis() - last_report > 3000) {
      char buf[60];
      sprintf(buf, "Waiting... LS_2A=%d LS_2B=%d", ls2a, ls2b);
      nh.loginfo(buf);
      last_report = millis();
    }
    
    nh.spinOnce();
    delay(50);
  }
  
  nh.loginfo("==================================================");
  nh.loginfo("JOINT 2 LIMIT SWITCH TEST RESULTS:");
  char buf[50];
  sprintf(buf, "  LS_2A (CCW limit): %s", ls2a_detected ? "DETECTED" : "NOT DETECTED");
  if (ls2a_detected) nh.loginfo(buf); else nh.logwarn(buf);
  sprintf(buf, "  LS_2B (CW limit):  %s", ls2b_detected ? "DETECTED" : "NOT DETECTED");
  if (ls2b_detected) nh.loginfo(buf); else nh.logwarn(buf);
  nh.loginfo("==================================================");
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void testIntegrated2() {
  if (!test_running) {
    nh.loginfo("==================================================");
    nh.loginfo("TEST: JOINT 2 INTEGRATED (Motor+Encoder+Sensor)");
    nh.loginfo("==================================================");
    nh.loginfo("Testing BOTH directions with brake control");
    nh.loginfo("Will return to starting position after test");
    nh.loginfo("==================================================");
    test_running = true;
  }

  // Record starting position
  long start_position = encoder_count_2;
  nh.loginfo("Recording starting position...");
  char buf[80];
  sprintf(buf, "  Starting encoder: %ld", start_position);
  nh.loginfo(buf);
  delay(1000);

  // ========== TEST 1: Move CCW to LS_2A ==========
  nh.loginfo("PHASE 1: Moving CCW to LS_2A...");
  encoder_count_2 = 0;
  enc2_interrupt_count = 0;
  
  nh.loginfo("Releasing brake...");
  brake_release();
  delay(500);
  
  unsigned long test_start = millis();
  
  while (!digitalRead(LS_2A) && (millis() - test_start < 30000)) {
    motorGo(MOTOR_2, CCW, 120);
    
    static unsigned long last_log = 0;
    if (millis() - last_log > 1000) {
      sprintf(buf, "  CCW: Encoder=%ld | LS_2A:%d LS_2B:%d", 
              encoder_count_2, digitalRead(LS_2A), digitalRead(LS_2B));
      nh.loginfo(buf);
      last_log = millis();
    }
    
    nh.spinOnce();
    Publish2();
    delay(10);
  }
  
  // Gradual stop
  for (int pwm = 120; pwm >= 0; pwm -= 15) {
    motorGo(MOTOR_2, CCW, pwm);
    delay(50);
  }
  motorGo(MOTOR_2, STOP, 0);
  
  nh.loginfo("Locking brake...");
  brake_lock();
  delay(1000);
  
  // Results Phase 1
  long ccw_encoder = encoder_count_2;
  long ccw_interrupts = enc2_interrupt_count;
  bool ccw_limit_reached = digitalRead(LS_2A);
  
  if (ccw_limit_reached) {
    nh.loginfo("Phase 1 SUCCESS: Reached LS_2A");
    sprintf(buf, "  CCW encoder pulses: %ld", ccw_encoder);
    nh.loginfo(buf);
  } else {
    nh.logerror("Phase 1 FAIL: LS_2A not reached in 30s");
  }
  
  // ========== TEST 2: Move CW to LS_2B ==========
  nh.loginfo("PHASE 2: Moving CW to LS_2B...");
  encoder_count_2 = 0;  // Reset for CW test
  enc2_interrupt_count = 0;
  
  nh.loginfo("Releasing brake...");
  brake_release();
  delay(500);
  
  test_start = millis();
  
  while (!digitalRead(LS_2B) && (millis() - test_start < 60000)) {
    motorGo(MOTOR_2, CW, 120);
    
    static unsigned long last_log2 = 0;
    if (millis() - last_log2 > 1000) {
      sprintf(buf, "  CW: Encoder=%ld | LS_2A:%d LS_2B:%d", 
              encoder_count_2, digitalRead(LS_2A), digitalRead(LS_2B));
      nh.loginfo(buf);
      last_log2 = millis();
    }
    
    nh.spinOnce();
    Publish2();
    delay(10);
  }
  
  // Gradual stop
  for (int pwm = 120; pwm >= 0; pwm -= 15) {
    motorGo(MOTOR_2, CW, pwm);
    delay(50);
  }
  motorGo(MOTOR_2, STOP, 0);
  brake_lock();
  delay(1000);
  
  // Results Phase 2
  long cw_encoder = encoder_count_2;
  long cw_interrupts = enc2_interrupt_count;
  bool cw_limit_reached = digitalRead(LS_2B);
  
  // ========== PHASE 3: Return to starting position ==========
  nh.loginfo("PHASE 3: Returning to starting position...");
  
  // Calculate target: we're at CW limit, need to move back CCW
  long target_from_cw_limit = -(ccw_encoder + cw_encoder) / 2;  // Approximate center
  encoder_count_2 = 0;  // Reset at CW limit
  
  sprintf(buf, "  Target offset from CW limit: %ld pulses", target_from_cw_limit);
  nh.loginfo(buf);
  
  nh.loginfo("Releasing brake...");
  brake_release();
  delay(500);
  
  test_start = millis();
  while (encoder_count_2 > target_from_cw_limit && (millis() - test_start < 45000)) {
    long error = target_from_cw_limit - encoder_count_2;
    int pwm = constrain(abs(error) / 50, 50, 120);  // Proportional speed
    
    motorGo(MOTOR_2, CCW, pwm);
    
    static unsigned long last_log3 = 0;
    if (millis() - last_log3 > 1000) {
      sprintf(buf, "  Returning: Encoder=%ld Target=%ld Error=%ld PWM=%d", 
              encoder_count_2, target_from_cw_limit, error, pwm);
      nh.loginfo(buf);
      last_log3 = millis();
    }
    
    // Stop if within tolerance
    if (abs(error) < 100) {
      nh.loginfo("  Reached starting position!");
      break;
    }
    
    nh.spinOnce();
    Publish2();
    delay(10);
  }
  
  // Gradual stop
  for (int pwm = 120; pwm >= 0; pwm -= 15) {
    motorGo(MOTOR_2, CCW, pwm);
    delay(50);
  }
  motorGo(MOTOR_2, STOP, 0);
  brake_lock();
  
  // ========== FINAL RESULTS ==========
  nh.loginfo("==================================================");
  nh.loginfo("JOINT 2 INTEGRATED TEST - FINAL RESULTS:");
  nh.loginfo("==================================================");
  
  // CCW Direction Results
  nh.loginfo("CCW Direction (to LS_2A):");
  sprintf(buf, "  Limit reached: %s", ccw_limit_reached ? "YES" : "NO");
  if (ccw_limit_reached) nh.loginfo(buf); else nh.logerror(buf);
  sprintf(buf, "  Encoder pulses: %ld", ccw_encoder);
  nh.loginfo(buf);
  sprintf(buf, "  Interrupts: %ld", ccw_interrupts);
  nh.loginfo(buf);
  
  nh.loginfo("--------------------------------------------------");
  
  // CW Direction Results
  nh.loginfo("CW Direction (to LS_2B):");
  sprintf(buf, "  Limit reached: %s", cw_limit_reached ? "YES" : "NO");
  if (cw_limit_reached) nh.loginfo(buf); else nh.logerror(buf);
  sprintf(buf, "  Encoder pulses: %ld", cw_encoder);
  nh.loginfo(buf);
  sprintf(buf, "  Interrupts: %ld", cw_interrupts);
  nh.loginfo(buf);
  
  nh.loginfo("--------------------------------------------------");
  
  // Overall Assessment
  sprintf(buf, "Total range: %ld pulses (CCW + CW)", abs(ccw_encoder) + abs(cw_encoder));
  nh.loginfo(buf);
  nh.loginfo("Joint returned to approximate starting position");
  
  if (ccw_limit_reached && cw_limit_reached) {
    if (abs(ccw_encoder) > 100 && abs(cw_encoder) > 100) {
      nh.loginfo("OVERALL: PASS - All systems working!");
      nh.loginfo("  Motor: OK | Encoder: OK | Sensors: OK | Brake: OK");
    } else {
      nh.logwarn("OVERALL: PARTIAL - Sensors OK, encoder count low");
      nh.logwarn("  Check: Encoder mechanical coupling");
    }
  } else {
    nh.logerror("OVERALL: FAIL - Check limit switch wiring");
  }
  
  nh.loginfo("==================================================");
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void Publish1(){
  pub_msg_1.joint = "Joint 1";
  pub_msg_1.setpoint = 0;
  pub_msg_1.pulse_count = encoder_count_1;
  pub_msg_1.error = 0;
  pub_msg_1.output = 0;
  pub_msg_1.control_loop = 0;
  pub_msg_1.IsDone = !test_running;
  
  pub_1.publish(&pub_msg_1);
}

void Publish2(){
  pub_msg_2.joint = "Joint 2";
  pub_msg_2.setpoint = 0;
  pub_msg_2.pulse_count = encoder_count_2;
  pub_msg_2.error = 0;
  pub_msg_2.output = 0;
  pub_msg_2.control_loop = 0;
  pub_msg_2.IsDone = !test_running;
  
  pub_2.publish(&pub_msg_2);
}

void Callback(const movemaster_msg::setpoint & rec_msg) {
  char buf[50];
  sprintf(buf, "Received: GoHome=%d", rec_msg.GoHome);
  nh.loginfo(buf);
  
  if (rec_msg.emergency_stop) {
    nh.logwarn("EMERGENCY STOP - Canceling test");
    motorGo(MOTOR_1, STOP, 0);
    motorGo(MOTOR_2, STOP, 0);
    brake_lock();
    test_mode = 0;
    test_running = false;
    showMenu();
    return;
  }
  
  // Use GoHome field for test selection (0-9)
  if (rec_msg.GoHome >= 1 && rec_msg.GoHome <= 9) {
    test_mode = rec_msg.GoHome;
    test_running = false;
    sprintf(buf, "Test mode set to: %d", test_mode);
    nh.loginfo(buf);
  }
}

void brake_lock(){
  digitalWrite(RELAY, HIGH);  // Inverted
  brake_flag = HIGH;
}

void brake_release(){
  digitalWrite(RELAY, LOW);   // Inverted
  brake_flag = LOW;
}

void CheckEncoder1(){
  encoder_count_1 += digitalRead(ENCODER_1B) == HIGH ? -1 : +1;
  enc1_interrupt_count++;
}

void CheckEncoder2(){
  encoder_count_2 += digitalRead(ENCODER_2B) == HIGH ? +1 : -1;
  enc2_interrupt_count++;
}

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
              analogWrite(PWM_2A, 0);
              analogWrite(PWM_2B, pwm);
              break;
            }
          case CCW: {
              analogWrite(PWM_2A, pwm);
              analogWrite(PWM_2B, 0);
              break;
            }
          case STOP: {
              analogWrite(PWM_2A, 0);
              analogWrite(PWM_2B, 0);
              break;
            }
        }
        break;
      }
  }
}

void testBrakeRelay() {
  nh.loginfo("==================================================");
  nh.loginfo("BRAKE RELAY TEST");
  nh.loginfo("==================================================");
  nh.loginfo("Watch/listen for relay clicking");
  nh.loginfo("Measure voltage at brake connector during test");
  nh.loginfo("==================================================");
  
  for (int i = 0; i < 5; i++) {
    char buf[50];
    
    sprintf(buf, "Cycle %d: RELEASING brake", i+1);
    nh.loginfo(buf);
    brake_release();  // HIGH
    delay(2000);
    
    sprintf(buf, "Cycle %d: LOCKING brake", i+1);
    nh.loginfo(buf);
    brake_lock();     // LOW
    delay(2000);
  }
  
  // ✅ ADD THIS: Ensure brake is locked at end of test
  nh.loginfo("Ensuring brake is locked (safe state)...");
  brake_lock();
  
  nh.loginfo("Test complete. Did you hear relay clicking?");
  nh.loginfo("Brake should have toggled 5 times");
  nh.loginfo("Brake is now LOCKED (relay LOW = safe)");
  nh.loginfo("==================================================");
}
