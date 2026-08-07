////////////////////////////////////////////////////////////////////////////////////////////
//  SIMPLE TEST SCRIPT - JOINTS 5 & 6
//  Upload to: /dev/arduino_3 (ttyACM0 or similar)
//
//  How to use:
//    1. roslaunch movemaster_control test_joints56.launch
//    2. Send test command from another terminal:
//
//       rostopic pub -1 /setpoints movemaster_msg/setpoint -- 0 0 0 0 0 false false <TEST>
//
//  Test codes (GoHome field):
//    1 = Joint 5 Motor     (CW → pause → CCW → stop)
//    2 = Joint 5 Encoder   (spin CW 3s and count pulses)
//    3 = Joint 5 Limits    (monitor switches 10s, manually press them)
//    4 = Joint 6 Motor     (OPEN → pause → CLOSE → stop)
//
//    emergency_stop = true  →  stop motors immediately
////////////////////////////////////////////////////////////////////////////////////////////

#include <ros.h>
#include <movemaster_msg/setpoint.h>
#include <movemaster_msg/status.h>

//--- PIN DEFINITIONS ---
#define STOP              0
#define CW                1
#define CCW               2
#define OPEN              3
#define CLOSE             4

#define HBRIDGE_5A        4
#define HBRIDGE_5B        9
#define HBRIDGE_6A        7
#define HBRIDGE_6B        8

#define PWM_5             6
#define PWM_6             5

#define ENABLE_5          A1
#define ENABLE_6          A0

#define MOTOR_5           5
#define MOTOR_6           6

#define ENCODER_5A        20
#define ENCODER_5B        21

#define LS_5A             34
#define LS_5B             36

//--- VARIABLES ---
volatile long enc5 = 0;
int test_mode = 0;
char buf[80];

//--- ROS ---
ros::NodeHandle nh;
movemaster_msg::status pub_msg_5, pub_msg_6;
ros::Publisher pub_5("/status_5", &pub_msg_5);
ros::Publisher pub_6("/status_6", &pub_msg_6);

void Callback(const movemaster_msg::setpoint &msg) {
  if (msg.emergency_stop) {
    motorGo(MOTOR_5, STOP, 0);
    motorGo(MOTOR_6, STOP, 0);
    test_mode = 0;
    nh.logwarn("EMERGENCY STOP - Motors stopped.");
    return;
  }
  test_mode = msg.GoHome;
}

ros::Subscriber<movemaster_msg::setpoint> sub("/setpoints", &Callback);


//--- SETUP ---
void setup() {
  pinMode(ENABLE_5,   OUTPUT); digitalWrite(ENABLE_5,  HIGH);
  pinMode(ENABLE_6,   OUTPUT); digitalWrite(ENABLE_6,  HIGH);

  pinMode(HBRIDGE_5A, OUTPUT); pinMode(HBRIDGE_5B, OUTPUT);
  pinMode(HBRIDGE_6A, OUTPUT); pinMode(HBRIDGE_6B, OUTPUT);

  pinMode(PWM_5,      OUTPUT);
  pinMode(PWM_6,      OUTPUT);

  pinMode(LS_5A, INPUT); pinMode(LS_5B, INPUT);

  // Encoder: INPUT_PULLUP (alterado para consistência de pull-up)
  pinMode(ENCODER_5A, INPUT_PULLUP);
  pinMode(ENCODER_5B, INPUT_PULLUP);

  // ISR no canal A com RISING (estado original)
  attachInterrupt(digitalPinToInterrupt(ENCODER_5A), isr5, RISING);

  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_5);
  nh.advertise(pub_6);

  nh.loginfo("Joints 5&6 test ready. Send GoHome=1..4 to run a test.");
}


//--- LOOP ---
void loop() {
  nh.spinOnce();

  if (!nh.connected()) {
    motorGo(MOTOR_5, STOP, 0);
    motorGo(MOTOR_6, STOP, 0);
    delay(100);
    return;
  }

  // Publish encoder counts as pulse_count for easy monitoring
  pub_msg_5.joint = "J5"; pub_msg_5.pulse_count = enc5;
  pub_msg_6.joint = "J6"; pub_msg_6.pulse_count = 0; // J6 doesnt have encoder
  pub_5.publish(&pub_msg_5);
  pub_6.publish(&pub_msg_6);

  switch (test_mode) {
    case 1: testMotor5();  test_mode = 0; break;
    case 2: testEncoder5();test_mode = 0; break;
    case 3: testLimits5(); test_mode = 0; break;
    case 4: testMotor6();  test_mode = 0; break;
    case 10:
    case 11: passivePinMonitor(); test_mode = 0; break;
    case 12: rawEncoderTest(); test_mode = 0; break;
    default: delay(50); break;
  }
}


//--- TESTS ---

void testMotor5() {
  nh.loginfo(">> TEST: Joint 5 Motor");

  // Log pin states before moving
  sprintf(buf, "   ENABLE_5(A1)=%d | HBRIDGE_5A(4)=%d | HBRIDGE_5B(9)=%d",
          digitalRead(ENABLE_5), digitalRead(HBRIDGE_5A), digitalRead(HBRIDGE_5B));
  nh.loginfo(buf);

  nh.loginfo("   CW for 2s (PWM=200)...");
  motorGo(MOTOR_5, CW, 200);
  delay(2000); nh.spinOnce();

  nh.loginfo("   Pause...");
  motorGo(MOTOR_5, STOP, 0); delay(1000); nh.spinOnce();

  nh.loginfo("   CCW for 2s (PWM=200)...");
  motorGo(MOTOR_5, CCW, 200);
  delay(2000); nh.spinOnce();

  motorGo(MOTOR_5, STOP, 0);
  nh.loginfo("   Done. Did the motor rotate in both directions?");
}

void testEncoder5() {
  nh.loginfo(">> TEST: Joint 5 Encoder");

  // --- Fase CW ---
  enc5 = 0;
  nh.loginfo("   [CW] Girando 3s...");
  unsigned long t = millis();
  while (millis() - t < 3000) {
    motorGo(MOTOR_5, CW, 200);
    nh.spinOnce(); delay(200);
    sprintf(buf, "   enc5 = %ld | LS_5A=%d LS_5B=%d", enc5, digitalRead(LS_5A), digitalRead(LS_5B));
    nh.loginfo(buf);
  }
  motorGo(MOTOR_5, STOP, 0);
  long cw_pulses = enc5;
  sprintf(buf, "   [CW] Resultado: %ld pulsos.", cw_pulses);
  nh.loginfo(buf);
  delay(500); nh.spinOnce();

  // --- Fase CCW ---
  enc5 = 0;
  nh.loginfo("   [CCW] Girando 3s...");
  t = millis();
  while (millis() - t < 3000) {
    motorGo(MOTOR_5, CCW, 200);
    nh.spinOnce(); delay(200);
    sprintf(buf, "   enc5 = %ld | LS_5A=%d LS_5B=%d", enc5, digitalRead(LS_5A), digitalRead(LS_5B));
    nh.loginfo(buf);
  }
  motorGo(MOTOR_5, STOP, 0);
  long ccw_pulses = enc5;
  sprintf(buf, "   [CCW] Resultado: %ld pulsos.", ccw_pulses);
  nh.loginfo(buf);

  // --- Avaliação ---
  nh.loginfo("   --- RESULTADO FINAL ---");
  bool cw_ok  = abs(cw_pulses)  > 50;
  bool ccw_ok = abs(ccw_pulses) > 50;
  bool opposite = (cw_pulses > 0 && ccw_pulses < 0) || (cw_pulses < 0 && ccw_pulses > 0);
  sprintf(buf, "   CW: %ld pulsos -> %s", cw_pulses,  cw_ok  ? "PASS" : "FALHOU");
  cw_ok  ? nh.loginfo(buf) : nh.logwarn(buf);
  sprintf(buf, "   CCW: %ld pulsos -> %s", ccw_pulses, ccw_ok ? "PASS" : "FALHOU");
  ccw_ok ? nh.loginfo(buf) : nh.logwarn(buf);
  sprintf(buf, "   Sentidos opostos: %s", opposite ? "SIM (OK)" : "NAO (verificar fiacao)");
  opposite ? nh.loginfo(buf) : nh.logwarn(buf);
}

void testLimits5() {
  nh.loginfo(">> TEST: Joint 5 Limit Switches (10s)");
  nh.loginfo("   Manually press LS_5A and LS_5B now...");
  bool saw_a = false, saw_b = false;
  unsigned long t = millis();
  while (millis() - t < 10000) {
    if (digitalRead(LS_5A) && !saw_a) { nh.loginfo("   LS_5A ACTIVATED!"); saw_a = true; }
    if (digitalRead(LS_5B) && !saw_b) { nh.loginfo("   LS_5B ACTIVATED!"); saw_b = true; }
    nh.spinOnce(); delay(100);
  }
  sprintf(buf, "   LS_5A: %s | LS_5B: %s", saw_a ? "OK" : "NOT SEEN", saw_b ? "OK" : "NOT SEEN");
  (saw_a && saw_b) ? nh.loginfo(buf) : nh.logwarn(buf);
}

void testMotor6() {
  nh.loginfo(">> TEST: Joint 6 (Gripper) Motor");
  nh.loginfo("   OPEN for 2s...");
  motorGo(MOTOR_6, OPEN, 255); delay(2000); nh.spinOnce();
  nh.loginfo("   Pause...");
  motorGo(MOTOR_6, STOP, 0); delay(1000); nh.spinOnce();
  nh.loginfo("   CLOSE for 2s...");
  motorGo(MOTOR_6, CLOSE, 255); delay(2000); nh.spinOnce();
  motorGo(MOTOR_6, STOP, 0);
  nh.loginfo("   Done. Did the gripper open and close?");
}

//--- HELPERS ---

void motorGo(int motor, int dir, int pwm) {
  if (motor == MOTOR_5) {
    digitalWrite(HBRIDGE_5A, dir == CW  ? HIGH : LOW);
    digitalWrite(HBRIDGE_5B, dir == CCW ? HIGH : LOW);
    analogWrite(PWM_5, pwm);
  } else if (motor == MOTOR_6) {
    digitalWrite(HBRIDGE_6A, dir == OPEN ? HIGH : LOW);
    digitalWrite(HBRIDGE_6B, dir == CLOSE ? HIGH : LOW);
    analogWrite(PWM_6, pwm);
  }
}

// Igual ao código de produção (Joints56.ino)
void isr5() { enc5 += (digitalRead(ENCODER_5B) == HIGH) ? -1 : +1; }

void passivePinMonitor() {
  nh.loginfo(">> PASSIVE MONITOR MODE - Listening to Encoders 18,19,20,21 for 10s");
  
  pinMode(18, INPUT_PULLUP); pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP); pinMode(21, INPUT_PULLUP);

  unsigned long t = millis();
  unsigned long last_log = millis();

  while (millis() - t < 10000) {
    if (millis() - last_log >= 500) {
      char buf[80];
      int s18 = digitalRead(18); int s19 = digitalRead(19);
      int s20 = digitalRead(20); int s21 = digitalRead(21);
      
      sprintf(buf, " t=%us (J56 PASSIVE) | P18=%d P19=%d P20=%d P21=%d",
              (unsigned int)((millis() - t) / 1000), s18, s19, s20, s21);
      nh.loginfo(buf);
      last_log = millis();
    }
    nh.spinOnce();
  }
}

// Diagnóstico bruto: roda motor 5s sem delay, reporta pulsos a cada 500ms
// Comando para testar: rostopic pub -1 /setpoints movemaster_msg/setpoint -- 0 0 0 0 0 false false 12
void rawEncoderTest() {
  nh.loginfo(">> TESTE RAW ENCODER - MONITORANDO TODOS OS ENCODERS (5s)");
  nh.loginfo("   Pinos Monitorados (A/B): J1(18/19), J2(20/21), J3(18/19), J4(20/21), J5(20/21)");
  
  // Configurando todos os pinos possíveis de encoder como INPUT_PULLUP
  pinMode(18, INPUT_PULLUP); pinMode(19, INPUT_PULLUP); // Pinos típicos EA/EB para J1 ou J3
  pinMode(20, INPUT_PULLUP); pinMode(21, INPUT_PULLUP); // Pinos típicos EA/EB para J2, J4 ou J5

  enc5 = 0;
  unsigned long t = millis();
  unsigned long last_log = millis();

  // Vamos girar o motor J5 para ver quem responde
  motorGo(MOTOR_5, CW, 200);

  while (millis() - t < 5000) {
    // Reporta a cada 500ms
    if (millis() - last_log >= 500) {
      int s18 = digitalRead(18);
      int s19 = digitalRead(19);
      int s20 = digitalRead(20);
      int s21 = digitalRead(21);
      
      sprintf(buf, " t=%us | P18=%d P19=%d P20=%d P21=%d | enc5=%ld",
              (unsigned int)((millis()-t)/1000), s18, s19, s20, s21, enc5);
      nh.loginfo(buf);

      last_log = millis();
      nh.spinOnce();  // comunica
    }
  }
  motorGo(MOTOR_5, STOP, 0);
  sprintf(buf, "   TOTAL: %ld pulsos em 5s", enc5);
  nh.loginfo(buf);
}
