////////////////////////////////////////////////////////////////////////////////
//  Joints56_vel — firmware de VELOCIDADE (malha aberta) para a placa Joints56
//  (J5 rolagem do punho, J6 gripper) do RV-M2 do b166er.          2026-10-07
//
//  Mesma lógica do Joints12_vel.ino (ler o cabeçalho de lá) para o J5:
//  velocidade em graus/s em set_5, sentido + PWM em malha aberta, zona morta,
//  saturação, fins de curso, watchdog, emergency_stop. Sem freio nesta placa.
//
//  GRIPPER (J6): não é junta de velocidade. set_GRIP = true fecha, false abre;
//  a placa aciona o motor da mão por GRIP_PULSE_MS a cada MUDANÇA de estado
//  (o mesmo pulso de 2 s do teste "código 4", que abriu e fechou em
//  06/10/2026) e depois desliga. Como a garra fica travada numa posição
//  durante a tarefa (ver garra_dx), na missão esse campo nem muda.
//
//  Pinos iguais aos do Joints56_test.ino (J5 e gripper validados em 06/10).
////////////////////////////////////////////////////////////////////////////////
#include <ros.h>
#include <movemaster_msg/setpoint.h>
#include <movemaster_msg/status.h>

// ---- pinos (= Joints56_test.ino) -------------------------------------------
#define HBRIDGE_5A   4
#define HBRIDGE_5B   9
#define HBRIDGE_6A   7
#define HBRIDGE_6B   8
#define PWM_5        6
#define PWM_6        5
#define ENABLE_5     A1
#define ENABLE_6     A0
#define LS_5A        34    // ativo em HIGH; A = lado CW, B = lado CCW
#define LS_5B        36

// ---- parâmetros (ajustar na bancada) ---------------------------------------
#define DIR_SIGN_5     (+1)
#define V_DEAD         1.5f
#define V_MAX          60.0f
#define PWM_MIN_5      50
#define PWM_MAX_5      200
#define K_PWM_5        2.5f
#define GRIP_PWM       255
#define GRIP_PULSE_MS  2000
#define WATCHDOG_MS    500
#define LOOP_HZ        50

ros::NodeHandle nh;
movemaster_msg::status st5, st6;
ros::Publisher pub5("status_5", &st5);
ros::Publisher pub6("status_6", &st6);

float v_cmd5 = 0.0f;
int   pwm_out5 = 0;
bool  grip_closed_cmd = false;     // último set_GRIP recebido
bool  grip_closed_state = false;   // estado que a placa acredita ter
unsigned long t_grip_pulse = 0;    // 0 = sem pulso em curso
int   grip_dir = 0;                // +1 fechar, -1 abrir
unsigned long t_last_msg = 0;
bool estop = false;

void callback(const movemaster_msg::setpoint &m) {
  t_last_msg = millis();
  if (m.emergency_stop) {
    estop = true; v_cmd5 = 0.0f;
    return;
  }
  estop = false;
  v_cmd5 = constrain(m.set_5, -V_MAX, V_MAX);
  grip_closed_cmd = m.set_GRIP;
}
ros::Subscriber<movemaster_msg::setpoint> sub("setpoints", &callback);

int pwmFor(float v, int pwm_min, int pwm_max, float k) {
  float a = fabs(v);
  if (a < V_DEAD) return 0;
  int p = (int)(pwm_min + k * a);
  if (p > pwm_max) p = pwm_max;
  return (v > 0) ? p : -p;
}

void driveJ5(int pwm) {
  if (pwm > 0 && digitalRead(LS_5A) == HIGH) pwm = 0;
  if (pwm < 0 && digitalRead(LS_5B) == HIGH) pwm = 0;
  digitalWrite(HBRIDGE_5A, pwm > 0 ? HIGH : LOW);
  digitalWrite(HBRIDGE_5B, pwm < 0 ? HIGH : LOW);
  analogWrite(PWM_5, abs(pwm));
  pwm_out5 = pwm;
}

void driveGrip(int dir, int pwm) {
  // dir > 0 fecha (CLOSE), dir < 0 abre (OPEN), 0 para — igual ao motorGo
  digitalWrite(HBRIDGE_6A, dir < 0 ? HIGH : LOW);   // OPEN
  digitalWrite(HBRIDGE_6B, dir > 0 ? HIGH : LOW);   // CLOSE
  analogWrite(PWM_6, dir == 0 ? 0 : pwm);
}

void updateGrip() {
  if (t_grip_pulse != 0) {
    if (millis() - t_grip_pulse >= GRIP_PULSE_MS) {
      driveGrip(0, 0);
      t_grip_pulse = 0;
      grip_closed_state = (grip_dir > 0);
    }
    return;
  }
  if (grip_closed_cmd != grip_closed_state && !estop) {
    grip_dir = grip_closed_cmd ? +1 : -1;
    driveGrip(grip_dir, GRIP_PWM);
    t_grip_pulse = millis();
    nh.loginfo(grip_closed_cmd ? "Joints56_vel: gripper FECHANDO (pulso 2 s)"
                               : "Joints56_vel: gripper ABRINDO (pulso 2 s)");
  }
}

void stopAll() {
  driveJ5(0);
  driveGrip(0, 0);
  t_grip_pulse = 0;
}

void publishStatus() {
  st5.joint = "J5"; st5.setpoint = v_cmd5; st5.pulse_count = 0; st5.error = 0;
  st5.output = pwm_out5; st5.control_loop = LOOP_HZ; st5.IsDone = (pwm_out5 == 0);
  st6.joint = "J6"; st6.setpoint = grip_closed_cmd ? 1 : 0; st6.pulse_count = 0; st6.error = 0;
  st6.output = (t_grip_pulse != 0) ? grip_dir * GRIP_PWM : 0; st6.control_loop = LOOP_HZ;
  st6.IsDone = (t_grip_pulse == 0);
  pub5.publish(&st5);
  pub6.publish(&st6);
}

void setup() {
  pinMode(ENABLE_5, OUTPUT); digitalWrite(ENABLE_5, HIGH);
  pinMode(ENABLE_6, OUTPUT); digitalWrite(ENABLE_6, HIGH);
  pinMode(HBRIDGE_5A, OUTPUT); pinMode(HBRIDGE_5B, OUTPUT);
  pinMode(HBRIDGE_6A, OUTPUT); pinMode(HBRIDGE_6B, OUTPUT);
  pinMode(PWM_5, OUTPUT); pinMode(PWM_6, OUTPUT);
  pinMode(LS_5A, INPUT); pinMode(LS_5B, INPUT);
  stopAll();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub5);
  nh.advertise(pub6);
  nh.loginfo("Joints56_vel: malha aberta, graus/s em set_5, gripper por set_GRIP, sem encoder");
}

void loop() {
  static unsigned long t_status = 0;
  nh.spinOnce();
  bool wd = (millis() - t_last_msg) > WATCHDOG_MS;
  if (estop || wd || t_last_msg == 0) {
    driveJ5(0);
    if (t_grip_pulse != 0 && estop) { driveGrip(0, 0); t_grip_pulse = 0; }
  } else {
    driveJ5(pwmFor(DIR_SIGN_5 * v_cmd5, PWM_MIN_5, PWM_MAX_5, K_PWM_5));
    updateGrip();
  }
  if (millis() - t_status >= 50) {
    t_status = millis();
    publishStatus();
  }
  delay(1000 / LOOP_HZ);
}
