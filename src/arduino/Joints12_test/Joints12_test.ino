////////////////////////////////////////////////////////////////////////////////////////////
//  SIMPLE TEST SCRIPT - JOINTS 1 & 2
//  Upload to: /dev/arduino_1 (ttyACM2)
//
//  How to use:
//    1. roslaunch movemaster_control test_joints12.launch
//    2. Send test command from another terminal:
//
//       rostopic pub -1 /setpoints movemaster_msg/setpoint -- 0 0 0 0 0 false
//       false <TEST>
//
//  Test codes (GoHome field):
//    1 = Joint 1 Motor     (CW → pause → CCW → stop)
//    2 = Joint 1 Encoder   (spin CW 3s and count pulses)
//    3 = Joint 1 Limits    (monitor switches 10s, manually press them)
//    4 = Joint 2 Motor     (CW → pause → CCW → stop)
//    5 = Joint 2 Encoder   (spin CW 3s and count pulses)
//    6 = Joint 2 Limits    (monitor switches 10s, manually press them)
//    7 = Joint 2 Brake OFF  (libera freio 30s para movimentação manual)
//
//    emergency_stop = true  →  stop motors immediately
////////////////////////////////////////////////////////////////////////////////////////////

#include <ros.h>
#include <movemaster_msg/setpoint.h>
#include <movemaster_msg/status.h>

//--- PIN DEFINITIONS ---
#define STOP 0
#define CW 1
#define CCW 2
#define MOTOR_1 1
#define MOTOR_2 2

#define HBRIDGE_1A 4 // J1 motor pin A
#define HBRIDGE_1B 9 // J1 motor pin B
#define HBRIDGE_2A 8 // J2 motor pin A
#define HBRIDGE_2B 7 // J2 motor pin B

#define PWM_1 6  // J1 PWM
#define PWM_2A 3 // J2 PWM A (BTS7960)
#define PWM_2B 2 // J2 PWM B (BTS7960)

#define ENABLE_1 A1  // J1 enable
#define ENABLE_2A 10 // J2 enable A
#define ENABLE_2B 11 // J2 enable B

#define ENCODER_1A 18
#define ENCODER_1B 19
#define ENCODER_2A 20
#define ENCODER_2B 21

#define LS_1A 34 // J1 limit switch A (CW  end)
#define LS_1B 36 // J1 limit switch B (CCW end)
#define LS_2A 38 // J2 limit switch A
#define LS_2B 40 // J2 limit switch B

#define RELAY 14 // J2 brake relay (HIGH = locked)

//--- VARIABLES ---
volatile long enc1 = 0;
volatile long enc2 = 0;
int test_mode = 0;
char buf[80];

//--- ROS ---
ros::NodeHandle nh;
movemaster_msg::status pub_msg_1, pub_msg_2;
ros::Publisher pub_1("/status_1", &pub_msg_1);
ros::Publisher pub_2("/status_2", &pub_msg_2);

void Callback(const movemaster_msg::setpoint &msg) {
  if (msg.emergency_stop) {
    motorGo(MOTOR_1, STOP, 0);
    motorGo(MOTOR_2, STOP, 0);
    digitalWrite(RELAY, HIGH); // brake locked
    test_mode = 0;
    nh.logwarn("EMERGENCY STOP - Motors stopped.");
    return;
  }
  test_mode = msg.GoHome;
}

ros::Subscriber<movemaster_msg::setpoint> sub("/setpoints", &Callback);

//--- SETUP ---
void setup() {
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH); // brake locked at boot
  pinMode(ENABLE_1, OUTPUT);
  digitalWrite(ENABLE_1, HIGH);
  pinMode(ENABLE_2A, OUTPUT);
  digitalWrite(ENABLE_2A, HIGH);
  pinMode(ENABLE_2B, OUTPUT);
  digitalWrite(ENABLE_2B, HIGH);
  pinMode(HBRIDGE_1A, OUTPUT);
  pinMode(HBRIDGE_1B, OUTPUT);
  pinMode(HBRIDGE_2A, OUTPUT);
  pinMode(HBRIDGE_2B, OUTPUT);
  pinMode(PWM_1, OUTPUT);
  pinMode(PWM_2A, OUTPUT);
  pinMode(PWM_2B, OUTPUT);
  pinMode(LS_1A, INPUT);
  pinMode(LS_1B, INPUT);
  pinMode(LS_2A, INPUT);
  pinMode(LS_2B, INPUT);
  // Encoder: INPUT_PULLUP para garantir sinal limpo
  pinMode(ENCODER_1A, INPUT);
  pinMode(ENCODER_1B, INPUT);
  pinMode(ENCODER_2A, INPUT);
  pinMode(ENCODER_2B, INPUT);

  // ISR no canal A com RISING (estado original)
  attachInterrupt(digitalPinToInterrupt(ENCODER_1A), isr1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2A), isr2, RISING);

  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_1);
  nh.advertise(pub_2);

  nh.loginfo("Joints 1&2 test ready. Send GoHome=1..6 to run a test.");
}

//--- LOOP ---
void loop() {
  nh.spinOnce();

  if (!nh.connected()) {
    motorGo(MOTOR_1, STOP, 0);
    motorGo(MOTOR_2, STOP, 0);
    digitalWrite(RELAY, HIGH);
    delay(100);
    return;
  }

  // Publish encoder counts as pulse_count for easy monitoring
  pub_msg_1.joint = "J1";
  pub_msg_1.pulse_count = enc1;
  pub_msg_2.joint = "J2";
  pub_msg_2.pulse_count = enc2;
  pub_1.publish(&pub_msg_1);
  pub_2.publish(&pub_msg_2);

  switch (test_mode) {
    case 1:
      testMotor1();
      test_mode = 0;
      break;
    case 2:
      testEncoder1();
      test_mode = 0;
      break;
    case 3:
      testLimits1();
      test_mode = 0;
      break;
    case 4:
      testMotor2();
      test_mode = 0;
      break;
    case 5:
      testEncoder2();
      test_mode = 0;
      break;
    case 6:
      testLimits2();
      test_mode = 0;
      break;
    case 7:
      manualBrakeRelease2();
      test_mode = 0;
      break;
    case 10:
      rawEncoderTest();
      test_mode = 0;
      break;
    case 11:
    case 12:
      passivePinMonitor();
      test_mode = 0;
      break;
    default:
      delay(50);
      break;
  }
}

void passivePinMonitor() {
  nh.loginfo(
    ">> PASSIVE MONITOR MODE - Listening to Encoders 18,19,20,21 for 10s");

  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);

  unsigned long t = millis();
  unsigned long last_log = millis();

  while (millis() - t < 10000) {
    if (millis() - last_log >= 500) {
      char buf[80];
      int s18 = digitalRead(18);
      int s19 = digitalRead(19);
      int s20 = digitalRead(20);
      int s21 = digitalRead(21);

      sprintf(buf, " t=%us (J12 PASSIVE) | P18=%d P19=%d P20=%d P21=%d",
              (unsigned int)((millis() - t) / 1000), s18, s19, s20, s21);
      nh.loginfo(buf);
      last_log = millis();
    }
    nh.spinOnce();
  }
}

//--- TESTS ---

void testMotor1() {
  nh.loginfo(">> TEST: Joint 1 Motor");

  // Log pin states before moving
  sprintf(buf, "   ENABLE_1(A1)=%d | HBRIDGE_1A(4)=%d | HBRIDGE_1B(9)=%d",
          digitalRead(A1), digitalRead(4), digitalRead(9));
  nh.loginfo(buf);

  nh.loginfo("   CW for 2s (PWM=200)...");
  motorGo(MOTOR_1, CW, 200);
  sprintf(buf, "   [CW] ENABLE=%d | 1A=%d | 1B=%d | PWM pin=6", digitalRead(A1),
          digitalRead(4), digitalRead(9));
  nh.loginfo(buf);
  delay(2000);
  nh.spinOnce();

  nh.loginfo("   Pause...");
  motorGo(MOTOR_1, STOP, 0);
  delay(1000);
  nh.spinOnce();

  nh.loginfo("   CCW for 2s (PWM=200)...");
  motorGo(MOTOR_1, CCW, 200);
  sprintf(buf, "   [CCW] ENABLE=%d | 1A=%d | 1B=%d", digitalRead(A1),
          digitalRead(4), digitalRead(9));
  nh.loginfo(buf);
  delay(2000);
  nh.spinOnce();

  motorGo(MOTOR_1, STOP, 0);
  nh.loginfo("   Done. Did the motor rotate in both directions?");
}

void testEncoder1() {
  nh.loginfo(">> TEST: Joint 1 Encoder");

  // --- Fase CW ---
  enc1 = 0;
  nh.loginfo("   [CW] Girando 3s...");
  unsigned long t = millis();
  while (millis() - t < 3000) {
    motorGo(MOTOR_1, CW, 200);
    nh.spinOnce();
    delay(200);
    sprintf(buf, "   enc1 = %ld | LS_1A=%d LS_1B=%d", enc1, digitalRead(LS_1A),
            digitalRead(LS_1B));
    nh.loginfo(buf);
  }
  motorGo(MOTOR_1, STOP, 0);
  long cw_pulses = enc1;
  sprintf(buf, "   [CW] Resultado: %ld pulsos.", cw_pulses);
  nh.loginfo(buf);
  delay(500);
  nh.spinOnce();

  // --- Fase CCW ---
  enc1 = 0;
  nh.loginfo("   [CCW] Girando 3s...");
  t = millis();
  while (millis() - t < 3000) {
    motorGo(MOTOR_1, CCW, 200);
    nh.spinOnce();
    delay(200);
    sprintf(buf, "   enc1 = %ld | LS_1A=%d LS_1B=%d", enc1, digitalRead(LS_1A),
            digitalRead(LS_1B));
    nh.loginfo(buf);
  }
  motorGo(MOTOR_1, STOP, 0);
  long ccw_pulses = enc1;
  sprintf(buf, "   [CCW] Resultado: %ld pulsos.", ccw_pulses);
  nh.loginfo(buf);

  // --- Avaliação ---
  nh.loginfo("   --- RESULTADO FINAL ---");
  bool cw_ok = abs(cw_pulses) > 200;
  bool ccw_ok = abs(ccw_pulses) > 200;
  bool opposite =
    (cw_pulses > 0 && ccw_pulses < 0) || (cw_pulses < 0 && ccw_pulses > 0);
  sprintf(buf, "   CW: %ld pulsos -> %s", cw_pulses, cw_ok ? "PASS" : "FALHOU");
  cw_ok ? nh.loginfo(buf) : nh.logwarn(buf);
  sprintf(buf, "   CCW: %ld pulsos -> %s", ccw_pulses,
          ccw_ok ? "PASS" : "FALHOU");
  ccw_ok ? nh.loginfo(buf) : nh.logwarn(buf);
  sprintf(buf, "   Sentidos opostos: %s",
          opposite ? "SIM (OK)" : "NAO (verificar fiacao)");
  opposite ? nh.loginfo(buf) : nh.logwarn(buf);
}

void testLimits1() {
  nh.loginfo(">> TEST: Joint 1 Limit Switches (10s)");
  nh.loginfo("   Manually press LS_1A and LS_1B now...");
  bool saw_a = false, saw_b = false;
  unsigned long t = millis();
  while (millis() - t < 10000) {
    if (digitalRead(LS_1A) && !saw_a) {
      nh.loginfo("   LS_1A ACTIVATED!");
      saw_a = true;
    }
    if (digitalRead(LS_1B) && !saw_b) {
      nh.loginfo("   LS_1B ACTIVATED!");
      saw_b = true;
    }
    nh.spinOnce();
    delay(100);
  }
  sprintf(buf, "   LS_1A: %s | LS_1B: %s", saw_a ? "OK" : "NOT SEEN",
          saw_b ? "OK" : "NOT SEEN");
  (saw_a && saw_b) ? nh.loginfo(buf) : nh.logwarn(buf);
}

void testMotor2() {
  nh.loginfo(">> TEST: Joint 2 Motor");
  digitalWrite(RELAY, LOW); // release brake
  delay(500);
  nh.loginfo("   CW for 2s...");
  motorGo(MOTOR_2, CW, 100);
  delay(2000);
  nh.spinOnce();
  nh.loginfo("   Pause...");
  motorGo(MOTOR_2, STOP, 0);
  delay(1000);
  nh.spinOnce();
  nh.loginfo("   CCW for 2s...");
  motorGo(MOTOR_2, CCW, 100);
  delay(2000);
  nh.spinOnce();
  motorGo(MOTOR_2, STOP, 0);
  digitalWrite(RELAY, HIGH); // lock brake
  nh.loginfo("   Done. Did the motor rotate in both directions?");
}

void testEncoder2() {
  nh.loginfo(">> TEST: Joint 2 Encoder");
  enc2 = 0;
  digitalWrite(RELAY, LOW); // release brake
  delay(500);
  nh.loginfo("   Spinning CW for 3s...");
  unsigned long t = millis();
  while (millis() - t < 3000) {
    motorGo(MOTOR_2, CW, 100);
    nh.spinOnce();
    delay(200);
    sprintf(buf, "   enc2 = %ld | LS_2A=%d LS_2B=%d", enc2, digitalRead(LS_2A),
            digitalRead(LS_2B));
    nh.loginfo(buf);
  }
  motorGo(MOTOR_2, STOP, 0);
  digitalWrite(RELAY, HIGH); // lock brake
  sprintf(buf, "   Result: %ld pulses. Expected >200 for PASS.", enc2);
  enc2 > 200 ? nh.loginfo(buf) : nh.logwarn(buf);
}

void testLimits2() {
  nh.loginfo(">> TEST: Joint 2 Limit Switches (10s)");
  nh.loginfo("   Manually press LS_2A and LS_2B now...");
  bool saw_a = false, saw_b = false;
  unsigned long t = millis();
  while (millis() - t < 10000) {
    if (digitalRead(LS_2A) && !saw_a) {
      nh.loginfo("   LS_2A ACTIVATED!");
      saw_a = true;
    }
    if (digitalRead(LS_2B) && !saw_b) {
      nh.loginfo("   LS_2B ACTIVATED!");
      saw_b = true;
    }
    nh.spinOnce();
    delay(100);
  }
  sprintf(buf, "   LS_2A: %s | LS_2B: %s", saw_a ? "OK" : "NOT SEEN",
          saw_b ? "OK" : "NOT SEEN");
  (saw_a && saw_b) ? nh.loginfo(buf) : nh.logwarn(buf);
}

void manualBrakeRelease2() {
  const unsigned long DURATION_MS = 30000UL; // 30 segundos
  enc2 = 0;
  motorGo(MOTOR_2, STOP, 0); // garante motor parado
  digitalWrite(RELAY, LOW);  // LIBERA o freio
  nh.loginfo(">> FREIO J2 LIBERADO - mova a junta manualmente (30s)");
  nh.loginfo("   Encoder enc2 zerado. Acompanhe os pulsos abaixo:");

  unsigned long t = millis();
  unsigned long last_log = millis();

  while (millis() - t < DURATION_MS) {
    nh.spinOnce();
    // Parada de emergência via ROS
    if (test_mode == 0 && digitalRead(RELAY) == LOW) {
      // test_mode já foi zerado: emergency_stop ativado
      break;
    }
    if (millis() - last_log >= 500) {
      unsigned long remaining = (DURATION_MS - (millis() - t)) / 1000;
      sprintf(buf, "   enc2=%ld | LS_2A=%d LS_2B=%d | restam ~%lus", enc2,
              digitalRead(LS_2A), digitalRead(LS_2B), remaining);
      nh.loginfo(buf);
      last_log = millis();
    }
  }

  digitalWrite(RELAY, HIGH); // TRAVA o freio novamente
  sprintf(buf, "   Freio travado. Deslocamento total: %ld pulsos.", enc2);
  nh.loginfo(buf);
  nh.loginfo("   Fim do teste manual J2.");
}

//--- HELPERS ---

void motorGo(int motor, int dir, int pwm) {
  if (motor == MOTOR_1) {
    digitalWrite(HBRIDGE_1A, dir == CW ? HIGH : LOW);
    digitalWrite(HBRIDGE_1B, dir == CCW ? HIGH : LOW);
    analogWrite(PWM_1, pwm);
  } else {
    analogWrite(PWM_2A, dir == CCW ? pwm : 0);
    analogWrite(PWM_2B, dir == CW ? pwm : 0);
  }
}

// Igual ao código de produção (Joints12.ino)
void isr1() {
  enc1 += (digitalRead(ENCODER_1B) == HIGH) ? -1 : +1;
}
void isr2() {
  enc2 += (digitalRead(ENCODER_2B) == HIGH) ? +1 : -1;
}

// Diagnóstico bruto: roda motor 5s, conta transições manualmente E via ISR
// Comando: rostopic pub -1 /setpoints movemaster_msg/setpoint -- 0 0 0 0 0 false false 10
void rawEncoderTest() {
  nh.loginfo(">> TESTE RAW ENCODER J1 (5s)");
  nh.loginfo("   Motor CW, PWM=200. Monitorando pinos 18(A) e 19(B).");
  nh.loginfo("   ISR enc1 + contagem manual de bordas em P18.");

  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);

  enc1 = 0;
  long manual_count = 0;        // conta transições manuais no pino 18

  int prev18 = digitalRead(18); // estado anterior do pino 18

  unsigned long t = millis();
  unsigned long last_log = millis();

  motorGo(MOTOR_1, CW, 200);

  while (millis() - t < 5000) {
    // Detecta borda de subida manualmente (sem ISR)
    int cur18 = digitalRead(18);
    if (cur18 == HIGH && prev18 == LOW) {
      manual_count++;
    }
    prev18 = cur18;

    // Reporta a cada 500ms
    if (millis() - last_log >= 500) {
      int s18 = digitalRead(18);
      int s19 = digitalRead(19);
      sprintf(buf, " t=%us | P18=%d P19=%d | ISR_enc1=%ld | manual=%ld",
              (unsigned int)((millis() - t) / 1000), s18, s19, enc1, manual_count);
      nh.loginfo(buf);
      last_log = millis();
      nh.spinOnce();
    }
  }
  motorGo(MOTOR_1, STOP, 0);

  nh.loginfo("   --- RESULTADO ---");
  sprintf(buf, "   ISR enc1   = %ld pulsos", enc1);
  enc1 > 10 ? nh.loginfo(buf) : nh.logwarn(buf);
  sprintf(buf, "   Manual P18 = %ld transicoes", manual_count);
  manual_count > 10 ? nh.loginfo(buf) : nh.logwarn(buf);

  if (manual_count > 10 && enc1 == 0)
    nh.logwarn("   >> Sinal OK mas ISR nao dispara! Verificar attachInterrupt.");
  else if (manual_count == 0)
    nh.logwarn("   >> Pino 18 sem transicoes! Verificar fiacao/encoder.");
  else
    nh.loginfo("   >> Encoder e ISR OK!");
}
