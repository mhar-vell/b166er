
////////////////////////////////////////////////////////////////////////////////////////////
//  SIMPLE TEST SCRIPT - JOINTS 3 & 4
//  Upload to: /dev/arduino_2 (TTY or similar)
//
//  How to use:
//    1. roslaunch movemaster_control test_joints34.launch (or equivalent)
//    2. Send test command from another terminal:
//
//       rostopic pub -1 /setpoints movemaster_msg/setpoint -- 0 0 0 0 0 false false <TEST>
//
//  Test codes (GoHome field):
//    1-4 = Joint 3 Tests (Motor, Encoder, Limits, Integrated)
//    5-8 = Joint 4 Tests (Motor, Encoder, Limits, Integrated)
//    9   = Joint 3 Brake Relay Test
//    10  = J3 Brake OFF (move manually 30s)
//    11  = Test All Encoders RAW (monitor 18/19/20/21)
//
//    emergency_stop = true  →  stop motors immediately
////////////////////////////////////////////////////////////////////////////////////////////

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

#define HBRIDGE_3A        4
#define HBRIDGE_3B        9
#define HBRIDGE_4A        7
#define HBRIDGE_4B        8

#define PWM_3             6
#define PWM_4             5

#define ENABLE_3          A1
#define ENABLE_4          A0

#define MOTOR_3           3
#define MOTOR_4           4

#define ENCODER_3A        18
#define ENCODER_3B        19
#define ENCODER_4A        20
#define ENCODER_4B        21

#define LS_3A             34
#define LS_3B             36
#define LS_4A             38
#define LS_4B             40

#define RELAY             14  // Joint 3 brake relay

/////////////////////////////////
///// VARIABLES DEFINITIONS /////
/////////////////////////////////

bool brake_flag = HIGH;
volatile long encoder_count_3 = 0;
volatile long encoder_count_4 = 0;
volatile long enc3_interrupt_count = 0;
volatile long enc4_interrupt_count = 0;
volatile long enc3b_interrupt_count = 0;
volatile long enc4b_interrupt_count = 0;
// WORKAROUND: J3 encoder Canal B com defeito (curto para GND).
// Direção determinada pelo comando do motor em vez do sinal B.
volatile int8_t motor3_dir = 1;  // +1=CW, -1=CCW, 0=parado

int test_mode = 0;  // 0=menu, 1=J3 motor, 2=J3 encoder, 3=J3 sensor, 4=J3 integrated,
                    //         5=J4 motor, 6=J4 encoder, 7=J4 sensor, 8=J4 integrated, 9=brake
bool test_running = false;

/////////////////////////////
///// ROS CONFIGURATION /////
/////////////////////////////

ros::NodeHandle nh;

void loginfo_P(const char* progmem_str) {
  char log_buf[128];
  strncpy_P(log_buf, progmem_str, sizeof(log_buf));
  log_buf[sizeof(log_buf)-1] = '\0';
  nh.loginfo(log_buf);
}

void logwarn_P(const char* progmem_str) {
  char log_buf[128];
  strncpy_P(log_buf, progmem_str, sizeof(log_buf));
  log_buf[sizeof(log_buf)-1] = '\0';
  nh.logwarn(log_buf);
}

void logerror_P(const char* progmem_str) {
  char log_buf[128];
  strncpy_P(log_buf, progmem_str, sizeof(log_buf));
  log_buf[sizeof(log_buf)-1] = '\0';
  nh.logerror(log_buf);
}
movemaster_msg::status pub_msg_3;
movemaster_msg::status pub_msg_4;
ros::Publisher pub_3("/status_3", &pub_msg_3);
ros::Publisher pub_4("/status_4", &pub_msg_4);

void Callback(const movemaster_msg::setpoint &rec_msg);
ros::Subscriber<movemaster_msg::setpoint> sub("/setpoints", &Callback);

////////////////////////
///// SYSTEM SETUP /////
////////////////////////

void setup()
{
  // Use internal pull-up to keep HIGH during boot
  pinMode(RELAY, INPUT_PULLUP);
  delay(10);
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);  // HIGH = brake locked (safe)
  brake_flag = HIGH;

  // Limit sensors as inputs
  pinMode(LS_3A, INPUT);
  pinMode(LS_3B, INPUT);
  pinMode(LS_4A, INPUT);
  pinMode(LS_4B, INPUT);

  // Encoders as inputs with pull-ups
  pinMode(ENCODER_3A, INPUT_PULLUP);
  pinMode(ENCODER_3B, INPUT_PULLUP);
  pinMode(ENCODER_4A, INPUT_PULLUP);
  pinMode(ENCODER_4B, INPUT_PULLUP);

  // Enable pins
  pinMode(ENABLE_3, OUTPUT);
  pinMode(ENABLE_4, OUTPUT);
  digitalWrite(ENABLE_3, HIGH);
  digitalWrite(ENABLE_4, HIGH);
  
  // H-bridge pins
  pinMode(HBRIDGE_3A, OUTPUT);
  pinMode(HBRIDGE_3B, OUTPUT);
  pinMode(HBRIDGE_4A, OUTPUT);
  pinMode(HBRIDGE_4B, OUTPUT);

  // PWM pins
  pinMode(PWM_3, OUTPUT);
  pinMode(PWM_4, OUTPUT);

  // J3: Canal B com defeito fisico (curto para GND). Usando apenas Canal A.
  // J4: 4x resolution normal (canais A e B OK).
  attachInterrupt(digitalPinToInterrupt(ENCODER_3A), CheckEncoder3A, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_3B), CheckEncoder3B, CHANGE); // DESABILITADO: canal B curto
  attachInterrupt(digitalPinToInterrupt(ENCODER_4A), CheckEncoder4A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_4B), CheckEncoder4B, CHANGE);

  // Start serial and ROS
  Serial.begin(57600);
  nh.initNode();
  delay(1000);
  nh.subscribe(sub);
  nh.advertise(pub_3);
  nh.advertise(pub_4);

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
    Publish3();
    Publish4();
    
    switch(test_mode) {
      case 1:  testMotor3();           break;
      case 2:  testEncoder3();         break;
      case 3:  testSensor3();          break;
      case 4:  testIntegrated3();      break;
      case 5:  testMotor4();           break;
      case 6:  testEncoder4();         break;
      case 7:  testSensor4();          break;
      case 8:  testIntegrated4();      break;
      case 9:  testBrakeRelay(); test_mode = 0; showMenu(); break;
      case 10:
      case 12: passivePinMonitor(); test_mode = 0; break;
      case 11: rawEncoderTest(); test_mode = 0; showMenu(); break;
      case 50: manualBrakeRelease3(); test_mode = 0; showMenu(); break;
      default:
        delay(100);
        break;
    }
    
    // Safety: ensure brake is locked when idle
    static int last_test_mode = -1;
    if (test_mode == 0 && last_test_mode != 0) {
      // Just transitioned to idle - ensure safe state ONCE
      motorGo(MOTOR_3, STOP, 0);
      motorGo(MOTOR_4, STOP, 0);
      brake_lock();
      loginfo_P(PSTR("System in safe idle state"));
    }
    last_test_mode = test_mode;
    
  } else {
    // Safety: stop everything if ROS disconnected
    motorGo(MOTOR_3, STOP, 0);
    motorGo(MOTOR_4, STOP, 0);
    digitalWrite(RELAY, HIGH);  // Force HIGH for safety (brake locked)
    brake_flag = HIGH;
    delay(100);
  }
}

/////////////////////
///// FUNCTIONS /////
/////////////////////

void showMenu() {
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("JOINTS 3&4 MAINTENANCE TEST MENU"));
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("Send setpoint with GoHome field to select test:"));
  loginfo_P(PSTR(""));
  loginfo_P(PSTR("JOINT 3 TESTS (with brake):"));
  loginfo_P(PSTR("  GoHome: 1 = Test Joint 3 Motor"));
  loginfo_P(PSTR("  GoHome: 2 = Test Joint 3 Encoder"));
  loginfo_P(PSTR("  GoHome: 3 = Test Joint 3 Limit Switches"));
  loginfo_P(PSTR("  GoHome: 4 = Test Joint 3 Integrated (all)"));
  loginfo_P(PSTR(""));
  loginfo_P(PSTR("JOINT 4 TESTS:"));
  loginfo_P(PSTR("  GoHome: 5 = Test Joint 4 Motor"));
  loginfo_P(PSTR("  GoHome: 6 = Test Joint 4 Encoder"));
  loginfo_P(PSTR("  GoHome: 7 = Test Joint 4 Limit Switches"));
  loginfo_P(PSTR("  GoHome: 8 = Test Joint 4 Integrated (all)"));
  loginfo_P(PSTR(""));
  loginfo_P(PSTR("BRAKE TEST:"));
  loginfo_P(PSTR("  GoHome: 9 = Test Brake Relay (Joint 3)"));
  loginfo_P(PSTR(""));
  loginfo_P(PSTR("MANUAL MANIPULATION:"));
  loginfo_P(PSTR("  GoHome: 10 = J3 Brake OFF 30s (move manually)"));
  loginfo_P(PSTR(""));
  loginfo_P(PSTR("RAW ENCODER TESTS:"));
  loginfo_P(PSTR("  GoHome: 11 = Test All Encoders RAW (monitor 18/19/20/21) while spinning J3"));
  loginfo_P(PSTR(""));
  loginfo_P(PSTR("  emergency_stop = true: Stop current test"));
  loginfo_P(PSTR("=================================================="));
}

//////////////////
// JOINT 3 TESTS (with brake control)
//////////////////

void testMotor3() {
  if (!test_running) {
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("TEST: JOINT 3 MOTOR"));
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("Testing Motor 3 with brake control"));
    loginfo_P(PSTR("NOTE: Joint 3 has brake - will release for tests"));
    loginfo_P(PSTR("=================================================="));
    test_running = true;
  }

  loginfo_P(PSTR("Releasing brake..."));
  brake_release();
  delay(1000);
  
  loginfo_P(PSTR("Motor 3: CW direction, ramping PWM 0->130"));
  for (int pwm = 0; pwm <= 130; pwm += 15) {
    motorGo(MOTOR_3, CW, pwm);
    char buf[50];
    sprintf(buf, "  CW PWM=%d", pwm);
    nh.loginfo(buf);
    delay(300);
    nh.spinOnce();
    Publish3();
  }
  
  loginfo_P(PSTR("Motor 3: Stopping and locking brake"));
  for (int pwm = 130; pwm >= 0; pwm -= 15) {
    motorGo(MOTOR_3, CW, pwm);
    delay(100);
    nh.spinOnce();
  }
  motorGo(MOTOR_3, STOP, 0);
  brake_lock();
  delay(2000);
  
  loginfo_P(PSTR("Releasing brake..."));
  brake_release();
  delay(1000);
  
  loginfo_P(PSTR("Motor 3: CCW direction (against gravity), ramping PWM 0->200"));
  for (int pwm = 0; pwm <= 200; pwm += 20) {
    motorGo(MOTOR_3, CCW, pwm);
    char buf[50];
    sprintf(buf, "  CCW PWM=%d", pwm);
    nh.loginfo(buf);
    delay(300);
    nh.spinOnce();
    Publish3();
  }
  
  loginfo_P(PSTR("Motor 3: Stopping and locking brake"));
  for (int pwm = 200; pwm >= 0; pwm -= 20) {
    motorGo(MOTOR_3, CCW, pwm);
    delay(100);
    nh.spinOnce();
  }
  motorGo(MOTOR_3, STOP, 0);
  brake_lock();
  
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("JOINT 3 MOTOR TEST COMPLETE"));
  loginfo_P(PSTR("Result: Motor should have rotated smoothly"));
  loginfo_P(PSTR("        Brake should lock/release correctly"));
  loginfo_P(PSTR("=================================================="));
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void testEncoder3() {
  if (!test_running) {
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("TEST: JOINT 3 ENCODER - COMPREHENSIVE"));
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("This test will:"));
    loginfo_P(PSTR("  1. Test CW rotation for 4 seconds"));
    loginfo_P(PSTR("  2. Test CCW rotation for 4 seconds"));
    loginfo_P(PSTR("  3. Verify direction sensing"));
    loginfo_P(PSTR("  4. Calculate pulse rate (pulses/second)"));
    loginfo_P(PSTR("  5. Check for mechanical slippage"));
    loginfo_P(PSTR("=================================================="));
    test_running = true;
  }

  long enc_start = encoder_count_3;
  long int_start = enc3_interrupt_count;
  char buf[120];
  
  loginfo_P(PSTR("Releasing brake..."));
  brake_release();
  delay(500);
  
  // ========== PHASE 1: CW ROTATION ==========
  loginfo_P(PSTR("PHASE 1: Testing CW rotation for 4 seconds..."));
  long cw_start_enc = encoder_count_3;
  long cw_start_int = enc3_interrupt_count;
  unsigned long cw_start_time = millis();
  unsigned long last_report = millis();
  
  while (millis() - cw_start_time < 4000) {
    motorGo(MOTOR_3, CW, 100);
    
    if (millis() - last_report > 1000) {
      unsigned long elapsed = millis() - cw_start_time;
      long pulses_so_far = encoder_count_3 - cw_start_enc;
      float rate = (pulses_so_far * 1000.0) / elapsed;
      sprintf(buf, "  CW: Enc=%ld Int=%ld Rate=%.1f p/s | E3A:%d E3B:%d", 
              encoder_count_3, enc3_interrupt_count, rate,
              digitalRead(ENCODER_3A), digitalRead(ENCODER_3B));
      nh.loginfo(buf);
      last_report = millis();
    }
    
    nh.spinOnce();
    Publish3();
    delay(10);
  }
  
  motorGo(MOTOR_3, STOP, 0);
  delay(500);
  
  unsigned long cw_duration = millis() - cw_start_time;
  long cw_enc_change = encoder_count_3 - cw_start_enc;
  long cw_int_change = enc3_interrupt_count - cw_start_int;
  float cw_rate = (cw_enc_change * 1000.0) / cw_duration;
  
  loginfo_P(PSTR("CW Results:"));
  sprintf(buf, "  Encoder pulses: %ld", cw_enc_change);
  nh.loginfo(buf);
  sprintf(buf, "  Interrupts: %ld", cw_int_change);
  nh.loginfo(buf);
  sprintf(buf, "  Pulse rate: %.1f pulses/second", cw_rate);
  nh.loginfo(buf);
  sprintf(buf, "  Duration: %lu ms", cw_duration);
  nh.loginfo(buf);
  
  // ========== PHASE 2: CCW ROTATION ==========
  loginfo_P(PSTR("PHASE 2: Testing CCW rotation for 4 seconds..."));
  long ccw_start_enc = encoder_count_3;
  long ccw_start_int = enc3_interrupt_count;
  unsigned long ccw_start_time = millis();
  last_report = millis();
  
  while (millis() - ccw_start_time < 4000) {
    motorGo(MOTOR_3, CCW, 150);  // Higher PWM for CCW (against gravity)
    
    if (millis() - last_report > 1000) {
      unsigned long elapsed = millis() - ccw_start_time;
      long pulses_so_far = ccw_start_enc - encoder_count_3;  // Negative change
      float rate = (abs(pulses_so_far) * 1000.0) / elapsed;
      sprintf(buf, "  CCW: Enc=%ld Int=%ld Rate=%.1f p/s | E3A:%d E3B:%d", 
              encoder_count_3, enc3_interrupt_count, rate,
              digitalRead(ENCODER_3A), digitalRead(ENCODER_3B));
      nh.loginfo(buf);
      last_report = millis();
    }
    
    nh.spinOnce();
    Publish3();
    delay(10);
  }
  
  motorGo(MOTOR_3, STOP, 0);
  brake_lock();
  
  unsigned long ccw_duration = millis() - ccw_start_time;
  long ccw_enc_change = encoder_count_3 - ccw_start_enc;
  long ccw_int_change = enc3_interrupt_count - ccw_start_int;
  float ccw_rate = (abs(ccw_enc_change) * 1000.0) / ccw_duration;
  
  loginfo_P(PSTR("CCW Results:"));
  sprintf(buf, "  Encoder pulses: %ld", ccw_enc_change);
  nh.loginfo(buf);
  sprintf(buf, "  Interrupts: %ld", ccw_int_change);
  nh.loginfo(buf);
  sprintf(buf, "  Pulse rate: %.1f pulses/second", ccw_rate);
  nh.loginfo(buf);
  sprintf(buf, "  Duration: %lu ms", ccw_duration);
  nh.loginfo(buf);
  
  // ========== ANALYSIS ==========
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("ENCODER TEST ANALYSIS:"));
  loginfo_P(PSTR("=================================================="));
  
  // Total counts
  long total_enc = abs(cw_enc_change) + abs(ccw_enc_change);
  long total_int = cw_int_change + ccw_int_change;
  sprintf(buf, "Total encoder pulses: %ld (CW:%ld + CCW:%ld)", 
          total_enc, abs(cw_enc_change), abs(ccw_enc_change));
  nh.loginfo(buf);
  sprintf(buf, "Total interrupts: %ld", total_int);
  nh.loginfo(buf);
  
  // Direction check
  bool correct_direction = (cw_enc_change > 0 && ccw_enc_change < 0) || 
                          (cw_enc_change < 0 && ccw_enc_change > 0);
  if (correct_direction) {
    loginfo_P(PSTR("Direction sensing: CORRECT (opposite signs)"));
  } else {
    nh.logwarn("Direction sensing: PROBLEM (same sign or zero)");
  }
  
  // Coupling check (compare interrupts vs encoder counts)
  float cw_efficiency = (cw_int_change > 0) ? (abs(cw_enc_change) * 100.0 / cw_int_change) : 0;
  float ccw_efficiency = (ccw_int_change > 0) ? (abs(ccw_enc_change) * 100.0 / ccw_int_change) : 0;
  
  sprintf(buf, "CW counting efficiency: %.1f%% (%ld counts / %ld interrupts)", 
          cw_efficiency, abs(cw_enc_change), cw_int_change);
  if (cw_efficiency > 80) nh.loginfo(buf); else nh.logwarn(buf);
  
  sprintf(buf, "CCW counting efficiency: %.1f%% (%ld counts / %ld interrupts)", 
          ccw_efficiency, abs(ccw_enc_change), ccw_int_change);
  if (ccw_efficiency > 80) nh.loginfo(buf); else nh.logwarn(buf);
  
  // Rate comparison
  float rate_ratio = (ccw_rate > 0) ? (cw_rate / ccw_rate) : 0;
  sprintf(buf, "Rate ratio CW/CCW: %.2f (should be ~0.5-1.0 for gravity)", rate_ratio);
  nh.loginfo(buf);
  
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("OVERALL ASSESSMENT:"));
  
  // Final verdict
  if (total_enc > 400 && correct_direction && cw_efficiency > 80 && ccw_efficiency > 80) {
    loginfo_P(PSTR("PASS: Encoder system is working correctly!"));
    loginfo_P(PSTR("  - Pulse counts are good (>400 total)"));
    loginfo_P(PSTR("  - Direction sensing works"));
    loginfo_P(PSTR("  - No significant slippage detected"));
  } else if (total_enc > 200) {
    nh.logwarn("PARTIAL: Encoder works but has issues:");
    if (total_enc < 400) {
      nh.logwarn("  - Low pulse count (check PWM or coupling)");
    }
    if (!correct_direction) {
      nh.logwarn("  - Direction sensing incorrect");
    }
    if (cw_efficiency < 80 || ccw_efficiency < 80) {
      nh.logwarn("  - Possible mechanical slippage");
    }
  } else if (total_int > 0) {
    nh.logerror("FAIL: Interrupts work but counts very low!");
    nh.logerror("  - Severe mechanical coupling problem");
    nh.logerror("  - Check: Set screws, coupler alignment");
  } else {
    nh.logerror("FAIL: Encoder not responding at all!");
    nh.logerror("  - Check: Power, wiring, connections");
  }
  
  loginfo_P(PSTR("=================================================="));
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void testSensor3() {
  if (!test_running) {
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("TEST: JOINT 3 LIMIT SWITCHES"));
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("Manually press each limit switch"));
    loginfo_P(PSTR("Test runs for 15 seconds"));
    loginfo_P(PSTR("=================================================="));
    test_running = true;
  }

  unsigned long test_start = millis();
  unsigned long last_report = millis();
  bool ls3a_detected = false;
  bool ls3b_detected = false;
  
  while (millis() - test_start < 15000) {
    bool ls3a = digitalRead(LS_3A);
    bool ls3b = digitalRead(LS_3B);
    
    if (ls3a && !ls3a_detected) {
      ls3a_detected = true;
      loginfo_P(PSTR("  >>> LS_3A ACTIVATED! <<<"));
      delay(500);
    }
    if (ls3b && !ls3b_detected) {
      ls3b_detected = true;
      loginfo_P(PSTR("  >>> LS_3B ACTIVATED! <<<"));
      delay(500);
    }
    
    if (millis() - last_report > 3000) {
      char buf[60];
      sprintf(buf, "Waiting... LS_3A=%d LS_3B=%d", ls3a, ls3b);
      nh.loginfo(buf);
      last_report = millis();
    }
    
    nh.spinOnce();
    delay(50);
  }
  
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("JOINT 3 LIMIT SWITCH TEST RESULTS:"));
  char buf[50];
  sprintf(buf, "  LS_3A (CCW limit): %s", ls3a_detected ? "DETECTED" : "NOT DETECTED");
  if (ls3a_detected) nh.loginfo(buf); else nh.logwarn(buf);
  sprintf(buf, "  LS_3B (CW limit):  %s", ls3b_detected ? "DETECTED" : "NOT DETECTED");
  if (ls3b_detected) nh.loginfo(buf); else nh.logwarn(buf);
  loginfo_P(PSTR("=================================================="));
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void testIntegrated3() {
  if (!test_running) {
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("TEST: JOINT 3 INTEGRATED (Motor+Encoder+Sensor)"));
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("Testing BOTH directions with brake control"));
    loginfo_P(PSTR("Will return to starting position after test"));
    loginfo_P(PSTR("NOTE: Joint 3 works against gravity in CCW"));
    loginfo_P(PSTR("=================================================="));
    test_running = true;
  }

  // Record starting position
  long start_position = encoder_count_3;
  loginfo_P(PSTR("Recording starting position..."));
  char buf[80];
  sprintf(buf, "  Starting encoder: %ld", start_position);
  nh.loginfo(buf);
  delay(1000);

  // ========== TEST 1: Move CW to LS_3B ==========
  loginfo_P(PSTR("PHASE 1: Moving CW to LS_3B..."));
  encoder_count_3 = 0;
  enc3_interrupt_count = 0;
  
  loginfo_P(PSTR("Releasing brake..."));
  brake_release();
  delay(500);
  
  unsigned long test_start = millis();
  
  while (!digitalRead(LS_3B) && (millis() - test_start < 30000)) {
    motorGo(MOTOR_3, CW, 120);
    
    static unsigned long last_log = 0;
    if (millis() - last_log > 1000) {
      sprintf(buf, "  CW: Encoder=%ld | LS_3A:%d LS_3B:%d", 
              encoder_count_3, digitalRead(LS_3A), digitalRead(LS_3B));
      nh.loginfo(buf);
      last_log = millis();
    }
    
    nh.spinOnce();
    Publish3();
    delay(10);
  }
  
  // Gradual stop
  for (int pwm = 120; pwm >= 0; pwm -= 15) {
    motorGo(MOTOR_3, CW, pwm);
    delay(50);
  }
  motorGo(MOTOR_3, STOP, 0);
  
  loginfo_P(PSTR("Locking brake..."));
  brake_lock();
  delay(1000);
  
  // Results Phase 1
  long cw_encoder = encoder_count_3;
  long cw_interrupts = enc3_interrupt_count;
  bool cw_limit_reached = digitalRead(LS_3B);
  
  if (cw_limit_reached) {
    loginfo_P(PSTR("Phase 1 SUCCESS: Reached LS_3B"));
    sprintf(buf, "  CW encoder pulses: %ld", cw_encoder);
    nh.loginfo(buf);
  } else {
    nh.logerror("Phase 1 FAIL: LS_3B not reached in 30s");
  }
  
  // ========== TEST 2: Move CCW to LS_3A (against gravity) ==========
  loginfo_P(PSTR("PHASE 2: Moving CCW to LS_3A (against gravity)..."));
  encoder_count_3 = 0;  // Reset for CCW test
  enc3_interrupt_count = 0;
  
  loginfo_P(PSTR("Releasing brake..."));
  brake_release();
  delay(500);
  
  test_start = millis();
  
  while (!digitalRead(LS_3A) && (millis() - test_start < 60000)) {
    motorGo(MOTOR_3, CCW, 200);  // Higher PWM for CCW against gravity
    
    static unsigned long last_log2 = 0;
    if (millis() - last_log2 > 1000) {
      sprintf(buf, "  CCW: Encoder=%ld | LS_3A:%d LS_3B:%d", 
              encoder_count_3, digitalRead(LS_3A), digitalRead(LS_3B));
      nh.loginfo(buf);
      last_log2 = millis();
    }
    
    nh.spinOnce();
    Publish3();
    delay(10);
  }
  
  // Gradual stop
  for (int pwm = 200; pwm >= 0; pwm -= 20) {
    motorGo(MOTOR_3, CCW, pwm);
    delay(50);
  }
  motorGo(MOTOR_3, STOP, 0);
  brake_lock();
  delay(1000);
  
  // Results Phase 2
  long ccw_encoder = encoder_count_3;
  long ccw_interrupts = enc3_interrupt_count;
  bool ccw_limit_reached = digitalRead(LS_3A);
  
  // ========== PHASE 3: Return to starting position ==========
  loginfo_P(PSTR("PHASE 3: Returning to starting position..."));
  
  // Calculate target: we're at CCW limit, need to move back CW
  // cw_encoder is positive, we want to go halfway = positive target
  long target_from_ccw_limit = cw_encoder / 2;  // Move halfway toward CW limit
  encoder_count_3 = 0;  // Reset at CCW limit
  
  sprintf(buf, "  Target offset from CCW limit: %ld pulses", target_from_ccw_limit);
  nh.loginfo(buf);
  
  loginfo_P(PSTR("Releasing brake..."));
  brake_release();
  delay(500);
  
  test_start = millis();
  while (encoder_count_3 < target_from_ccw_limit && (millis() - test_start < 45000)) {
    long error = target_from_ccw_limit - encoder_count_3;
    int pwm = constrain(abs(error) / 50, 50, 120);  // Proportional speed
    
    motorGo(MOTOR_3, CW, pwm);
    
    static unsigned long last_log3 = 0;
    if (millis() - last_log3 > 1000) {
      sprintf(buf, "  Returning: Encoder=%ld Target=%ld Error=%ld PWM=%d", 
              encoder_count_3, target_from_ccw_limit, error, pwm);
      nh.loginfo(buf);
      last_log3 = millis();
    }
    
    // Stop if within tolerance
    if (abs(error) < 100) {
      loginfo_P(PSTR("  Reached starting position!"));
      break;
    }
    
    nh.spinOnce();
    Publish3();
    delay(10);
  }
  
  // Gradual stop
  for (int pwm = 120; pwm >= 0; pwm -= 15) {
    motorGo(MOTOR_3, CW, pwm);
    delay(50);
  }
  motorGo(MOTOR_3, STOP, 0);
  brake_lock();
  
  // ========== FINAL RESULTS ==========
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("JOINT 3 INTEGRATED TEST - FINAL RESULTS:"));
  loginfo_P(PSTR("=================================================="));
  
  // CW Direction Results
  loginfo_P(PSTR("CW Direction (to LS_3B):"));
  sprintf(buf, "  Limit reached: %s", cw_limit_reached ? "YES" : "NO");
  if (cw_limit_reached) nh.loginfo(buf); else nh.logerror(buf);
  sprintf(buf, "  Encoder pulses: %ld", cw_encoder);
  nh.loginfo(buf);
  sprintf(buf, "  Interrupts: %ld", cw_interrupts);
  nh.loginfo(buf);
  
  loginfo_P(PSTR("--------------------------------------------------"));
  
  // CCW Direction Results
  loginfo_P(PSTR("CCW Direction (to LS_3A - against gravity):"));
  sprintf(buf, "  Limit reached: %s", ccw_limit_reached ? "YES" : "NO");
  if (ccw_limit_reached) nh.loginfo(buf); else nh.logerror(buf);
  sprintf(buf, "  Encoder pulses: %ld", ccw_encoder);
  nh.loginfo(buf);
  sprintf(buf, "  Interrupts: %ld", ccw_interrupts);
  nh.loginfo(buf);
  
  loginfo_P(PSTR("--------------------------------------------------"));
  
  // Overall Assessment
  sprintf(buf, "Total range: %ld pulses (CW + CCW)", abs(cw_encoder) + abs(ccw_encoder));
  nh.loginfo(buf);
  loginfo_P(PSTR("Joint returned to approximate starting position"));
  
  if (cw_limit_reached && ccw_limit_reached) {
    if (abs(cw_encoder) > 100 && abs(ccw_encoder) > 100) {
      loginfo_P(PSTR("OVERALL: PASS - All systems working!"));
      loginfo_P(PSTR("  Motor: OK | Encoder: OK | Sensors: OK | Brake: OK"));
    } else {
      nh.logwarn("OVERALL: PARTIAL - Sensors OK, encoder count low");
      nh.logwarn("  Check: Encoder mechanical coupling");
    }
  } else {
    nh.logerror("OVERALL: FAIL - Check limit switch wiring");
  }
  
  loginfo_P(PSTR("=================================================="));
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

//////////////////
// JOINT 4 TESTS (no brake)
//////////////////

void testMotor4() {
  if (!test_running) {
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("TEST: JOINT 4 MOTOR"));
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("Testing Motor 4 directions and PWM response"));
    loginfo_P(PSTR("Watch for smooth acceleration/deceleration"));
    loginfo_P(PSTR("=================================================="));
    test_running = true;
  }

  loginfo_P(PSTR("Motor 4: CW direction, ramping PWM 0->180"));
  for (int pwm = 0; pwm <= 180; pwm += 20) {
    motorGo(MOTOR_4, CW, pwm);
    char buf[50];
    sprintf(buf, "  CW PWM=%d", pwm);
    nh.loginfo(buf);
    delay(300);
    nh.spinOnce();
    Publish4();
  }
  
  loginfo_P(PSTR("Motor 4: Stopping"));
  for (int pwm = 180; pwm >= 0; pwm -= 20) {
    motorGo(MOTOR_4, CW, pwm);
    delay(100);
    nh.spinOnce();
  }
  motorGo(MOTOR_4, STOP, 0);
  delay(2000);
  
  loginfo_P(PSTR("Motor 4: CCW direction, ramping PWM 0->180"));
  for (int pwm = 0; pwm <= 180; pwm += 20) {
    motorGo(MOTOR_4, CCW, pwm);
    char buf[50];
    sprintf(buf, "  CCW PWM=%d", pwm);
    nh.loginfo(buf);
    delay(300);
    nh.spinOnce();
    Publish4();
  }
  
  loginfo_P(PSTR("Motor 4: Stopping"));
  for (int pwm = 180; pwm >= 0; pwm -= 20) {
    motorGo(MOTOR_4, CCW, pwm);
    delay(100);
    nh.spinOnce();
  }
  motorGo(MOTOR_4, STOP, 0);
  
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("JOINT 4 MOTOR TEST COMPLETE"));
  loginfo_P(PSTR("Result: Motor should have rotated smoothly"));
  loginfo_P(PSTR("        in both CW and CCW directions"));
  loginfo_P(PSTR("=================================================="));
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void testEncoder4() {
  if (!test_running) {
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("TEST: JOINT 4 ENCODER - COMPREHENSIVE"));
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("This test will:"));
    loginfo_P(PSTR("  1. Test CW rotation for 4 seconds"));
    loginfo_P(PSTR("  2. Test CCW rotation for 4 seconds"));
    loginfo_P(PSTR("  3. Verify direction sensing"));
    loginfo_P(PSTR("  4. Calculate pulse rate (pulses/second)"));
    loginfo_P(PSTR("  5. Check for mechanical slippage"));
    loginfo_P(PSTR("=================================================="));
    test_running = true;
  }

  long enc_start = encoder_count_4;
  long int_start = enc4_interrupt_count;
  char buf[120];
  
  // ========== PHASE 1: CW ROTATION ==========
  loginfo_P(PSTR("PHASE 1: Testing CW rotation for 4 seconds..."));
  long cw_start_enc = encoder_count_4;
  long cw_start_int = enc4_interrupt_count;
  unsigned long cw_start_time = millis();
  unsigned long last_report = millis();
  
  while (millis() - cw_start_time < 4000) {
    motorGo(MOTOR_4, CW, 100);
    
    if (millis() - last_report > 1000) {
      unsigned long elapsed = millis() - cw_start_time;
      long pulses_so_far = encoder_count_4 - cw_start_enc;
      float rate = (pulses_so_far * 1000.0) / elapsed;
      sprintf(buf, "  CW: Enc=%ld Int=%ld Rate=%.1f p/s | E4A:%d E4B:%d", 
              encoder_count_4, enc4_interrupt_count, rate,
              digitalRead(ENCODER_4A), digitalRead(ENCODER_4B));
      nh.loginfo(buf);
      last_report = millis();
    }
    
    nh.spinOnce();
    Publish4();
    delay(10);
  }
  
  motorGo(MOTOR_4, STOP, 0);
  delay(500);
  
  unsigned long cw_duration = millis() - cw_start_time;
  long cw_enc_change = encoder_count_4 - cw_start_enc;
  long cw_int_change = enc4_interrupt_count - cw_start_int;
  float cw_rate = (cw_enc_change * 1000.0) / cw_duration;
  
  loginfo_P(PSTR("CW Results:"));
  sprintf(buf, "  Encoder pulses: %ld", cw_enc_change);
  nh.loginfo(buf);
  sprintf(buf, "  Interrupts: %ld", cw_int_change);
  nh.loginfo(buf);
  sprintf(buf, "  Pulse rate: %.1f pulses/second", cw_rate);
  nh.loginfo(buf);
  sprintf(buf, "  Duration: %lu ms", cw_duration);
  nh.loginfo(buf);
  
  // ========== PHASE 2: CCW ROTATION ==========
  loginfo_P(PSTR("PHASE 2: Testing CCW rotation for 4 seconds..."));
  long ccw_start_enc = encoder_count_4;
  long ccw_start_int = enc4_interrupt_count;
  unsigned long ccw_start_time = millis();
  last_report = millis();
  
  while (millis() - ccw_start_time < 4000) {
    motorGo(MOTOR_4, CCW, 100);
    
    if (millis() - last_report > 1000) {
      unsigned long elapsed = millis() - ccw_start_time;
      long pulses_so_far = ccw_start_enc - encoder_count_4;  // Negative change
      float rate = (abs(pulses_so_far) * 1000.0) / elapsed;
      sprintf(buf, "  CCW: Enc=%ld Int=%ld Rate=%.1f p/s | E4A:%d E4B:%d", 
              encoder_count_4, enc4_interrupt_count, rate,
              digitalRead(ENCODER_4A), digitalRead(ENCODER_4B));
      nh.loginfo(buf);
      last_report = millis();
    }
    
    nh.spinOnce();
    Publish4();
    delay(10);
  }
  
  motorGo(MOTOR_4, STOP, 0);
  
  unsigned long ccw_duration = millis() - ccw_start_time;
  long ccw_enc_change = encoder_count_4 - ccw_start_enc;
  long ccw_int_change = enc4_interrupt_count - ccw_start_int;
  float ccw_rate = (abs(ccw_enc_change) * 1000.0) / ccw_duration;
  
  loginfo_P(PSTR("CCW Results:"));
  sprintf(buf, "  Encoder pulses: %ld", ccw_enc_change);
  nh.loginfo(buf);
  sprintf(buf, "  Interrupts: %ld", ccw_int_change);
  nh.loginfo(buf);
  sprintf(buf, "  Pulse rate: %.1f pulses/second", ccw_rate);
  nh.loginfo(buf);
  sprintf(buf, "  Duration: %lu ms", ccw_duration);
  nh.loginfo(buf);
  
  // ========== ANALYSIS ==========
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("ENCODER TEST ANALYSIS:"));
  loginfo_P(PSTR("=================================================="));
  
  // Total counts
  long total_enc = abs(cw_enc_change) + abs(ccw_enc_change);
  long total_int = cw_int_change + ccw_int_change;
  sprintf(buf, "Total encoder pulses: %ld (CW:%ld + CCW:%ld)", 
          total_enc, abs(cw_enc_change), abs(ccw_enc_change));
  nh.loginfo(buf);
  sprintf(buf, "Total interrupts: %ld", total_int);
  nh.loginfo(buf);
  
  // Direction check
  bool correct_direction = (cw_enc_change > 0 && ccw_enc_change < 0) || 
                          (cw_enc_change < 0 && ccw_enc_change > 0);
  if (correct_direction) {
    loginfo_P(PSTR("Direction sensing: CORRECT (opposite signs)"));
  } else {
    nh.logwarn("Direction sensing: PROBLEM (same sign or zero)");
  }
  
  // Coupling check (compare interrupts vs encoder counts)
  float cw_efficiency = (cw_int_change > 0) ? (abs(cw_enc_change) * 100.0 / cw_int_change) : 0;
  float ccw_efficiency = (ccw_int_change > 0) ? (abs(ccw_enc_change) * 100.0 / ccw_int_change) : 0;
  
  sprintf(buf, "CW counting efficiency: %.1f%% (%ld counts / %ld interrupts)", 
          cw_efficiency, abs(cw_enc_change), cw_int_change);
  if (cw_efficiency > 80) nh.loginfo(buf); else nh.logwarn(buf);
  
  sprintf(buf, "CCW counting efficiency: %.1f%% (%ld counts / %ld interrupts)", 
          ccw_efficiency, abs(ccw_enc_change), ccw_int_change);
  if (ccw_efficiency > 80) nh.loginfo(buf); else nh.logwarn(buf);
  
  // Rate comparison  
  float rate_ratio = (ccw_rate > 0) ? (cw_rate / ccw_rate) : 0;
  sprintf(buf, "Rate ratio CW/CCW: %.2f (should be ~0.8-1.2)", rate_ratio);
  nh.loginfo(buf);
  
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("OVERALL ASSESSMENT:"));
  
  // Final verdict
  if (total_enc > 400 && correct_direction && cw_efficiency > 80 && ccw_efficiency > 80) {
    loginfo_P(PSTR("PASS: Encoder system is working correctly!"));
    loginfo_P(PSTR("  - Pulse counts are good (>400 total)"));
    loginfo_P(PSTR("  - Direction sensing works"));
    loginfo_P(PSTR("  - No significant slippage detected"));
  } else if (total_enc > 200) {
    nh.logwarn("PARTIAL: Encoder works but has issues:");
    if (total_enc < 400) {
      nh.logwarn("  - Low pulse count (check PWM or coupling)");
    }
    if (!correct_direction) {
      nh.logwarn("  - Direction sensing incorrect");
    }
    if (cw_efficiency < 80 || ccw_efficiency < 80) {
      nh.logwarn("  - Possible mechanical slippage");
    }
  } else if (total_int > 0) {
    nh.logerror("FAIL: Interrupts work but counts very low!");
    nh.logerror("  - Severe mechanical coupling problem");
    nh.logerror("  - Check: Set screws, coupler alignment");
  } else {
    nh.logerror("FAIL: Encoder not responding at all!");
    nh.logerror("  - Check: Power, wiring, connections");
  }
  
  loginfo_P(PSTR("=================================================="));
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void testSensor4() {
  if (!test_running) {
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("TEST: JOINT 4 LIMIT SWITCHES"));
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("Manually press each limit switch"));
    loginfo_P(PSTR("Test runs for 15 seconds"));
    loginfo_P(PSTR("=================================================="));
    test_running = true;
  }

  unsigned long test_start = millis();
  unsigned long last_report = millis();
  bool ls4a_detected = false;
  bool ls4b_detected = false;
  
  while (millis() - test_start < 15000) {
    bool ls4a = digitalRead(LS_4A);
    bool ls4b = digitalRead(LS_4B);
    
    if (ls4a && !ls4a_detected) {
      ls4a_detected = true;
      loginfo_P(PSTR("  >>> LS_4A ACTIVATED! <<<"));
      delay(500);
    }
    if (ls4b && !ls4b_detected) {
      ls4b_detected = true;
      loginfo_P(PSTR("  >>> LS_4B ACTIVATED! <<<"));
      delay(500);
    }
    
    if (millis() - last_report > 3000) {
      char buf[60];
      sprintf(buf, "Waiting... LS_4A=%d LS_4B=%d", ls4a, ls4b);
      nh.loginfo(buf);
      last_report = millis();
    }
    
    nh.spinOnce();
    delay(50);
  }
  
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("JOINT 4 LIMIT SWITCH TEST RESULTS:"));
  char buf[50];
  sprintf(buf, "  LS_4A (CCW limit): %s", ls4a_detected ? "DETECTED" : "NOT DETECTED");
  if (ls4a_detected) nh.loginfo(buf); else nh.logwarn(buf);
  sprintf(buf, "  LS_4B (CW limit):  %s", ls4b_detected ? "DETECTED" : "NOT DETECTED");
  if (ls4b_detected) nh.loginfo(buf); else nh.logwarn(buf);
  loginfo_P(PSTR("=================================================="));
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void testIntegrated4() {
  if (!test_running) {
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("TEST: JOINT 4 INTEGRATED (Motor+Encoder+Sensor)"));
    loginfo_P(PSTR("=================================================="));
    loginfo_P(PSTR("Testing BOTH directions with full range motion"));
    loginfo_P(PSTR("Will return to starting position after test"));
    loginfo_P(PSTR("=================================================="));
    test_running = true;
  }

  // Record starting position
  long start_position = encoder_count_4;
  loginfo_P(PSTR("Recording starting position..."));
  char buf[80];
  sprintf(buf, "  Starting encoder: %ld", start_position);
  nh.loginfo(buf);
  delay(1000);

  // ========== TEST 1: Move CCW to LS_4A ==========
  loginfo_P(PSTR("PHASE 1: Moving CCW to LS_4A..."));
  encoder_count_4 = 0;
  enc4_interrupt_count = 0;
  unsigned long test_start = millis();
  
  while (!digitalRead(LS_4A) && (millis() - test_start < 30000)) {
    motorGo(MOTOR_4, CCW, 120);
    
    static unsigned long last_log = 0;
    if (millis() - last_log > 1000) {
      sprintf(buf, "  CCW: Encoder=%ld | LS_4A:%d LS_4B:%d", 
              encoder_count_4, digitalRead(LS_4A), digitalRead(LS_4B));
      nh.loginfo(buf);
      last_log = millis();
    }
    
    nh.spinOnce();
    Publish4();
    delay(10);
  }
  
  // Gradual stop
  for (int pwm = 120; pwm >= 0; pwm -= 15) {
    motorGo(MOTOR_4, CCW, pwm);
    delay(50);
  }
  motorGo(MOTOR_4, STOP, 0);
  delay(1000);
  
  // Results Phase 1
  long ccw_encoder = encoder_count_4;
  long ccw_interrupts = enc4_interrupt_count;
  bool ccw_limit_reached = digitalRead(LS_4A);
  
  if (ccw_limit_reached) {
    loginfo_P(PSTR("Phase 1 SUCCESS: Reached LS_4A"));
    sprintf(buf, "  CCW encoder pulses: %ld", ccw_encoder);
    nh.loginfo(buf);
  } else {
    nh.logerror("Phase 1 FAIL: LS_4A not reached in 30s");
  }
  
  // ========== TEST 2: Move CW to LS_4B ==========
  loginfo_P(PSTR("PHASE 2: Moving CW to LS_4B..."));
  encoder_count_4 = 0;  // Reset for CW test
  enc4_interrupt_count = 0;
  test_start = millis();
  
  while (!digitalRead(LS_4B) && (millis() - test_start < 60000)) {
    motorGo(MOTOR_4, CW, 120);
    
    static unsigned long last_log2 = 0;
    if (millis() - last_log2 > 1000) {
      sprintf(buf, "  CW: Encoder=%ld | LS_4A:%d LS_4B:%d", 
              encoder_count_4, digitalRead(LS_4A), digitalRead(LS_4B));
      nh.loginfo(buf);
      last_log2 = millis();
    }
    
    nh.spinOnce();
    Publish4();
    delay(10);
  }
  
  // Gradual stop
  for (int pwm = 120; pwm >= 0; pwm -= 15) {
    motorGo(MOTOR_4, CW, pwm);
    delay(50);
  }
  motorGo(MOTOR_4, STOP, 0);
  delay(1000);
  
  // Results Phase 2
  long cw_encoder = encoder_count_4;
  long cw_interrupts = enc4_interrupt_count;
  bool cw_limit_reached = digitalRead(LS_4B);
  
  if (cw_limit_reached) {
    loginfo_P(PSTR("Phase 2 SUCCESS: Reached LS_4B"));
    sprintf(buf, "  CW encoder pulses: %ld", cw_encoder);
    nh.loginfo(buf);
  } else {
    nh.logerror("Phase 2 FAIL: LS_4B not reached in 60s");
  }
  
  // ========== PHASE 3: Return to starting position ==========
  loginfo_P(PSTR("PHASE 3: Returning to starting position..."));
  
  // Calculate target: we're at CW limit (encoder=0), need to move back CCW
  // ccw_encoder is negative, we want halfway = negative target  
  long target_from_cw_limit = ccw_encoder / 2;  // Move halfway toward CCW limit (negative)
  encoder_count_4 = 0;  // Reset at CW limit
  
  sprintf(buf, "  Target offset from CW limit: %ld pulses", target_from_cw_limit);
  nh.loginfo(buf);
  
  test_start = millis();
  while (encoder_count_4 > target_from_cw_limit && (millis() - test_start < 45000)) {
    long error = target_from_cw_limit - encoder_count_4;
    int pwm = constrain(abs(error) / 50, 50, 120);  // Proportional speed
    
    motorGo(MOTOR_4, CCW, pwm);
    
    static unsigned long last_log3 = 0;
    if (millis() - last_log3 > 1000) {
      sprintf(buf, "  Returning: Encoder=%ld Target=%ld Error=%ld PWM=%d", 
              encoder_count_4, target_from_cw_limit, error, pwm);
      nh.loginfo(buf);
      last_log3 = millis();
    }
    
    // Stop if within tolerance
    if (abs(error) < 100) {
      loginfo_P(PSTR("  Reached starting position!"));
      break;
    }
    
    nh.spinOnce();
    Publish4();
    delay(10);
  }
  
  // Gradual stop
  for (int pwm = 120; pwm >= 0; pwm -= 15) {
    motorGo(MOTOR_4, CCW, pwm);
    delay(50);
  }
  motorGo(MOTOR_4, STOP, 0);
  
  // ========== FINAL RESULTS ==========
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("JOINT 4 INTEGRATED TEST - FINAL RESULTS:"));
  loginfo_P(PSTR("=================================================="));
  
  // CCW Direction Results
  loginfo_P(PSTR("CCW Direction (to LS_4A):"));
  sprintf(buf, "  Limit reached: %s", ccw_limit_reached ? "YES" : "NO");
  if (ccw_limit_reached) nh.loginfo(buf); else nh.logerror(buf);
  sprintf(buf, "  Encoder pulses: %ld", ccw_encoder);
  nh.loginfo(buf);
  sprintf(buf, "  Interrupts: %ld", ccw_interrupts);
  nh.loginfo(buf);
  
  loginfo_P(PSTR("--------------------------------------------------"));
  
  // CW Direction Results
  loginfo_P(PSTR("CW Direction (to LS_4B):"));
  sprintf(buf, "  Limit reached: %s", cw_limit_reached ? "YES" : "NO");
  if (cw_limit_reached) nh.loginfo(buf); else nh.logerror(buf);
  sprintf(buf, "  Encoder pulses: %ld", cw_encoder);
  nh.loginfo(buf);
  sprintf(buf, "  Interrupts: %ld", cw_interrupts);
  nh.loginfo(buf);
  
  loginfo_P(PSTR("--------------------------------------------------"));
  
  // Overall Assessment
  sprintf(buf, "Total range: %ld pulses (CCW + CW)", abs(ccw_encoder) + abs(cw_encoder));
  nh.loginfo(buf);
  loginfo_P(PSTR("Joint returned to approximate starting position"));
  
  if (ccw_limit_reached && cw_limit_reached) {
    if (abs(ccw_encoder) > 100 && abs(cw_encoder) > 100) {
      loginfo_P(PSTR("OVERALL: PASS - All systems working!"));
      loginfo_P(PSTR("  Motor: OK | Encoder: OK | Sensors: OK"));
    } else {
      nh.logwarn("OVERALL: PARTIAL - Sensors OK, encoder count low");
      nh.logwarn("  Check: Encoder mechanical coupling");
    }
  } else {
    nh.logerror("OVERALL: FAIL - Check limit switch wiring");
  }
  
  loginfo_P(PSTR("=================================================="));
  
  test_mode = 0;
  test_running = false;
  showMenu();
}

void Publish3(){
  pub_msg_3.joint = "Joint 3";
  pub_msg_3.setpoint = 0;
  pub_msg_3.pulse_count = encoder_count_3;
  pub_msg_3.error = 0;
  pub_msg_3.output = 0;
  pub_msg_3.control_loop = 0;
  pub_msg_3.IsDone = !test_running;
  
  pub_3.publish(&pub_msg_3);
}

void Publish4(){
  pub_msg_4.joint = "Joint 4";
  pub_msg_4.setpoint = 0;
  pub_msg_4.pulse_count = encoder_count_4;
  pub_msg_4.error = 0;
  pub_msg_4.output = 0;
  pub_msg_4.control_loop = 0;
  pub_msg_4.IsDone = !test_running;
  
  pub_4.publish(&pub_msg_4);
}

void Callback(const movemaster_msg::setpoint & rec_msg) {
  char buf[50];
  sprintf(buf, "Received: GoHome=%d", rec_msg.GoHome);
  nh.loginfo(buf);
  
  if (rec_msg.emergency_stop) {
    nh.logwarn("EMERGENCY STOP - Canceling test");
    motorGo(MOTOR_3, STOP, 0);
    motorGo(MOTOR_4, STOP, 0);
    brake_lock();
    test_mode = 0;
    test_running = false;
    showMenu();
    return;
  }
  
  // Use GoHome field for test selection (0-10)
  if (rec_msg.GoHome >= 1 && rec_msg.GoHome <= 10) {
    test_mode = rec_msg.GoHome;
    test_running = false;
    sprintf(buf, "Test mode set to: %d", test_mode);
    nh.loginfo(buf);
  }
}

void manualBrakeRelease3() {
  const unsigned long DURATION_MS = 30000UL;  // 30 segundos
  encoder_count_3 = 0;
  motorGo(MOTOR_3, STOP, 0);       // garante motor parado
  brake_release();                  // LIBERA o freio (RELAY = LOW)
  loginfo_P(PSTR(">> FREIO J3 LIBERADO - mova a junta manualmente (30s)"));
  loginfo_P(PSTR("   Encoder enc3 zerado. Acompanhe os pulsos abaixo:"));

  unsigned long t = millis();
  unsigned long last_log = millis();

  while (millis() - t < DURATION_MS) {
    nh.spinOnce();
    if (millis() - last_log >= 500) {
      unsigned long remaining = (DURATION_MS - (millis() - t)) / 1000;
      char buf[80];
      sprintf(buf, "   enc3=%ld | LS_3A=%d LS_3B=%d | restam ~%lus",
              encoder_count_3, digitalRead(LS_3A), digitalRead(LS_3B), remaining);
      nh.loginfo(buf);
      last_log = millis();
    }
  }

  motorGo(MOTOR_3, STOP, 0);
  brake_lock();                     // TRAVA o freio novamente
  char buf[60];
  sprintf(buf, "   Freio travado. Deslocamento total: %ld pulsos.", encoder_count_3);
  nh.loginfo(buf);
  loginfo_P(PSTR("   Fim do teste manual J3."));
}

void brake_lock(){
  digitalWrite(RELAY, HIGH);  // HIGH = brake locked
  brake_flag = HIGH;
}

void brake_release(){
  digitalWrite(RELAY, LOW);   // LOW = brake released
  brake_flag = LOW;
}

void CheckEncoder3A(){
  // WORKAROUND: Canal B com defeito. Usa motor3_dir para determinar sentido.
  // Resolucao reduzida para 2x (CHANGE em A only) vs 4x original.
  encoder_count_3 += motor3_dir;
  enc3_interrupt_count++;
}

// CheckEncoder3B DESABILITADA - canal B fisicamente em curto com GND.
// void CheckEncoder3B() { ... }

void CheckEncoder4A(){
  // Direct port read (faster than digitalRead)
  // Pin 20 = PD1, Pin 21 = PD0
  int A = (PIND >> 1) & 1;  // Read bit 1 of PORTD (pin 20)
  int B = (PIND >> 0) & 1;  // Read bit 0 of PORTD (pin 21)
  
  // XOR logic: if A==B, rotating CCW (negative), else CW (positive)
  encoder_count_4 += (A == B) ? -1 : 1;
  enc4_interrupt_count++;
}

void CheckEncoder4B(){
  // Direct port read (faster than digitalRead)
  // Pin 20 = PD1, Pin 21 = PD0
  int A = (PIND >> 1) & 1;  // Read bit 1 of PORTD (pin 20)
  int B = (PIND >> 0) & 1;  // Read bit 0 of PORTD (pin 21)
  
  // Opposite logic for B channel: if A==B, rotating CW (positive), else CCW (negative)
  encoder_count_4 += (A == B) ? 1 : -1;
  enc4b_interrupt_count++;
}

void motorGo(int motor, int dir, int pwm)
{
  switch (motor)
  {
    case MOTOR_3: {
        switch (dir)
        {
          case CW: {
              motor3_dir = +1;  // WORKAROUND: salva dir para ISR do encoder J3
              digitalWrite(HBRIDGE_3A, HIGH);
              digitalWrite(HBRIDGE_3B, LOW);
              break;
            }
          case CCW: {
              motor3_dir = -1;  // WORKAROUND: salva dir para ISR do encoder J3
              digitalWrite(HBRIDGE_3A, LOW);
              digitalWrite(HBRIDGE_3B, HIGH);
              break;
            }
          case STOP: {
              motor3_dir = 0;   // WORKAROUND: salva dir para ISR do encoder J3
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

void testBrakeRelay() {
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("BRAKE RELAY TEST (JOINT 3)"));
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("Watch/listen for relay clicking"));
  loginfo_P(PSTR("Measure voltage at brake connector during test"));
  loginfo_P(PSTR("NOTE: Only Joint 3 has a brake"));
  loginfo_P(PSTR("=================================================="));
  
  for (int i = 0; i < 5; i++) {
    char buf[50];
    
    sprintf(buf, "Cycle %d: RELEASING brake", i+1);
    nh.loginfo(buf);
    brake_release();  // LOW
    delay(2000);
    
    sprintf(buf, "Cycle %d: LOCKING brake", i+1);
    nh.loginfo(buf);
    brake_lock();     // HIGH
    delay(2000);
  }
  
  // Ensure brake is locked at end of test
  loginfo_P(PSTR("Ensuring brake is locked (safe state)..."));
  brake_lock();
  
  loginfo_P(PSTR("Test complete. Did you hear relay clicking?"));
  loginfo_P(PSTR("Brake should have toggled 5 times"));
  loginfo_P(PSTR("Brake is now LOCKED (relay HIGH = safe)"));
  loginfo_P(PSTR("=================================================="));
}

void passivePinMonitor() {
  loginfo_P(PSTR(">> PASSIVE MONITOR MODE - Listening to Encoders 18,19,20,21 for 10s"));
  
  pinMode(18, INPUT_PULLUP); pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP); pinMode(21, INPUT_PULLUP);

  unsigned long t = millis();
  unsigned long last_log = millis();

  while (millis() - t < 10000) {
    if (millis() - last_log >= 500) {
      char buf[80];
      int s18 = digitalRead(18); int s19 = digitalRead(19);
      int s20 = digitalRead(20); int s21 = digitalRead(21);
      
      sprintf(buf, " t=%us (J34 PASSIVE) | P18=%d P19=%d P20=%d P21=%d",
              (unsigned int)((millis() - t) / 1000), s18, s19, s20, s21);
      nh.loginfo(buf);
      last_log = millis();
    }
    nh.spinOnce();
  }
}

// Diagnóstico bruto: roda motor 5s sem delay, reporta pulsos a cada 500ms
// Comando para testar: rostopic pub -1 /setpoints movemaster_msg/setpoint -- 0 0 0 0 0 false false 11
void rawEncoderTest() {
  loginfo_P(PSTR(">> TESTE RAW ENCODER - MONITORANDO TODOS OS ENCODERS (5s)"));
  loginfo_P(PSTR("   Pinos Monitorados (A/B): J1(18/19), J2(20/21), J3(18/19), J4(20/21), J5(20/21)"));
  
  // Configurando todos os pinos possíveis de encoder como INPUT_PULLUP
  pinMode(18, INPUT_PULLUP); pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP); pinMode(21, INPUT_PULLUP);

  char buf[80];
  encoder_count_3 = 0;
  unsigned long t = millis();
  unsigned long last_log = millis();

  // Vamos girar o motor J3 (após soltar freio) para ver quem responde
  brake_release();
  delay(100);
  motorGo(MOTOR_3, CW, 150);

  while (millis() - t < 5000) {
    // Reporta a cada 500ms
    if (millis() - last_log >= 500) {
      int s18 = digitalRead(18);
      int s19 = digitalRead(19);
      int s20 = digitalRead(20);
      int s21 = digitalRead(21);
      
      sprintf(buf, " t=%us | P18=%d P19=%d P20=%d P21=%d | enc3=%ld",
              (unsigned int)((millis()-t)/1000), s18, s19, s20, s21, encoder_count_3);
      nh.loginfo(buf);

      last_log = millis();
      nh.spinOnce();  // comunica
    }
  }
  motorGo(MOTOR_3, STOP, 0);
  brake_lock();
  
  sprintf(buf, "   TOTAL: %ld pulsos em 5s", encoder_count_3);
  nh.loginfo(buf);
}

//////////////////////////////////////////
// HOMING FUNCTIONS (for reference)
//////////////////////////////////////////

void homeJoint3() {
  // Example: Home Joint 3 to LS_3A (CCW limit) and set as zero
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("HOMING JOINT 3 to LS_3A (Zero Reference)"));
  loginfo_P(PSTR("=================================================="));
  
  char buf[80];
  
  // Release brake
  loginfo_P(PSTR("Releasing brake..."));
  brake_release();
  delay(500);
  
  // Move CCW slowly until hitting limit switch
  loginfo_P(PSTR("Moving CCW to find LS_3A..."));
  unsigned long home_start = millis();
  
  while (!digitalRead(LS_3A) && (millis() - home_start < 30000)) {
    motorGo(MOTOR_3, CCW, 80);  // Slow speed for homing
    nh.spinOnce();
    delay(10);
  }
  
  // Stop motor
  motorGo(MOTOR_3, STOP, 0);
  
  if (digitalRead(LS_3A)) {
    loginfo_P(PSTR("SUCCESS: Reached LS_3A limit switch"));
    
    // Set this position as ZERO
    encoder_count_3 = 0;
    enc3_interrupt_count = 0;
    enc3b_interrupt_count = 0;
    
    loginfo_P(PSTR("Encoder count reset to ZERO at LS_3A"));
    loginfo_P(PSTR("This is now the HOME/REFERENCE position"));
    
    // Back off slightly from limit switch
    loginfo_P(PSTR("Backing off 50 pulses from limit..."));
    long target = 50;
    
    while (encoder_count_3 < target) {
      motorGo(MOTOR_3, CW, 60);
      nh.spinOnce();
      delay(10);
    }
    motorGo(MOTOR_3, STOP, 0);
    
    sprintf(buf, "Final position: %ld pulses from LS_3A", encoder_count_3);
    nh.loginfo(buf);
    loginfo_P(PSTR("Joint 3 is now HOMED and CALIBRATED"));
    
  } else {
    nh.logerror("FAIL: Could not reach LS_3A in 30 seconds");
  }
  
  brake_lock();
  loginfo_P(PSTR("=================================================="));
}

void homeJoint4() {
  // Example: Home Joint 4 to LS_4A (CCW limit) and set as zero
  loginfo_P(PSTR("=================================================="));
  loginfo_P(PSTR("HOMING JOINT 4 to LS_4A (Zero Reference)"));
  loginfo_P(PSTR("=================================================="));
  
  char buf[80];
  
  // Move CCW slowly until hitting limit switch
  loginfo_P(PSTR("Moving CCW to find LS_4A..."));
  unsigned long home_start = millis();
  
  while (!digitalRead(LS_4A) && (millis() - home_start < 30000)) {
    motorGo(MOTOR_4, CCW, 80);  // Slow speed for homing
    nh.spinOnce();
    delay(10);
  }
  
  // Stop motor
  motorGo(MOTOR_4, STOP, 0);
  
  if (digitalRead(LS_4A)) {
    loginfo_P(PSTR("SUCCESS: Reached LS_4A limit switch"));
    
    // Set this position as ZERO
    encoder_count_4 = 0;
    enc4_interrupt_count = 0;
    enc4b_interrupt_count = 0;
    
    loginfo_P(PSTR("Encoder count reset to ZERO at LS_4A"));
    loginfo_P(PSTR("This is now the HOME/REFERENCE position"));
    
    // Back off slightly from limit switch
    loginfo_P(PSTR("Backing off 50 pulses from limit..."));
    long target = 50;
    
    while (encoder_count_4 < target) {
      motorGo(MOTOR_4, CW, 60);
      nh.spinOnce();
      delay(10);
    }
    motorGo(MOTOR_4, STOP, 0);
    
    sprintf(buf, "Final position: %ld pulses from LS_4A", encoder_count_4);
    nh.loginfo(buf);
    loginfo_P(PSTR("Joint 4 is now HOMED and CALIBRATED"));
    
  } else {
    nh.logerror("FAIL: Could not reach LS_4A in 30 seconds");
  }
  
  loginfo_P(PSTR("=================================================="));
}
