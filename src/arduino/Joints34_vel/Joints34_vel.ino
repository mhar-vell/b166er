////////////////////////////////////////////////////////////////////////////////
//  Joints34_vel — firmware de VELOCIDADE (malha aberta) para a placa Joints34
//  (J3 cotovelo, J4 inclinação do punho) do RV-M2 do b166er.      2026-10-07
//
//  Mesma lógica do Joints12_vel.ino (ler o cabeçalho de lá): velocidade em
//  graus/s por junta em set_3/set_4, sentido + PWM em malha aberta, zona
//  morta, saturação, fins de curso, watchdog, emergency_stop. Aqui o freio é
//  o do J3 (RELAY, HIGH = travado); o J4 não tem freio. Pinos iguais aos do
//  Joints34_test.ino (J4 girou nos dois sentidos com o código 5 em 06/10/2026).
//
//  Atenção ao J4: o URDF limita o esforço a 4,2 N·m e a bancada (RELATORIO11)
//  mostrou que o punho cede no libera; PWM_MAX_4 mais baixo que o dos outros
//  eixos de propósito — subir só com medição.
////////////////////////////////////////////////////////////////////////////////
#include <ros.h>
#include <movemaster_msg/setpoint.h>
#include <movemaster_msg/status.h>

// ---- pinos (= Joints34_test.ino) -------------------------------------------
#define HBRIDGE_3A   4
#define HBRIDGE_3B   9
#define HBRIDGE_4A   7
#define HBRIDGE_4B   8
#define PWM_3        6
#define PWM_4        5
#define ENABLE_3     A1
#define ENABLE_4     A0
#define LS_3A        34    // ativo em HIGH; A = lado CW, B = lado CCW
#define LS_3B        36
#define LS_4A        38
#define LS_4B        40
#define RELAY        14    // freio do J3: HIGH = travado

// ---- parâmetros (ajustar na bancada) ---------------------------------------
#define DIR_SIGN_3    (+1)
#define DIR_SIGN_4    (+1)
#define V_DEAD        1.5f
#define V_MAX         60.0f
#define PWM_MIN_3     70
#define PWM_MIN_4     50
#define PWM_MAX_3     230
#define PWM_MAX_4     180
#define K_PWM_3       2.6f
#define K_PWM_4       2.1f
#define BRAKE_LOCK_MS 300
#define WATCHDOG_MS   500
#define LOOP_HZ       50

ros::NodeHandle nh;
movemaster_msg::status st3, st4;
ros::Publisher pub3("status_3", &st3);
ros::Publisher pub4("status_4", &st4);

float v_cmd[2] = {0.0f, 0.0f};     // J3, J4
int   pwm_out[2] = {0, 0};
unsigned long t_last_msg = 0;
unsigned long t_j3_zero  = 0;
bool estop = false;
bool brake_locked = true;

void brake3(bool lock) {
  if (lock != brake_locked) {
    digitalWrite(RELAY, lock ? HIGH : LOW);
    brake_locked = lock;
  }
}

void callback(const movemaster_msg::setpoint &m) {
  t_last_msg = millis();
  if (m.emergency_stop) {
    estop = true; v_cmd[0] = 0.0f; v_cmd[1] = 0.0f;
    return;
  }
  estop = false;
  v_cmd[0] = constrain(m.set_3, -V_MAX, V_MAX);
  v_cmd[1] = constrain(m.set_4, -V_MAX, V_MAX);
}
ros::Subscriber<movemaster_msg::setpoint> sub("setpoints", &callback);

int pwmFor(float v, int pwm_min, int pwm_max, float k) {
  float a = fabs(v);
  if (a < V_DEAD) return 0;
  int p = (int)(pwm_min + k * a);
  if (p > pwm_max) p = pwm_max;
  return (v > 0) ? p : -p;
}

void driveJ3(int pwm) {
  if (pwm > 0 && digitalRead(LS_3A) == HIGH) pwm = 0;
  if (pwm < 0 && digitalRead(LS_3B) == HIGH) pwm = 0;
  if (pwm != 0) {
    brake3(false);
    t_j3_zero = 0;
  } else {
    if (t_j3_zero == 0) t_j3_zero = millis();
    if (millis() - t_j3_zero >= BRAKE_LOCK_MS) brake3(true);
  }
  digitalWrite(HBRIDGE_3A, pwm > 0 ? HIGH : LOW);
  digitalWrite(HBRIDGE_3B, pwm < 0 ? HIGH : LOW);
  analogWrite(PWM_3, abs(pwm));
  pwm_out[0] = pwm;
}

void driveJ4(int pwm) {
  if (pwm > 0 && digitalRead(LS_4A) == HIGH) pwm = 0;
  if (pwm < 0 && digitalRead(LS_4B) == HIGH) pwm = 0;
  digitalWrite(HBRIDGE_4A, pwm > 0 ? HIGH : LOW);
  digitalWrite(HBRIDGE_4B, pwm < 0 ? HIGH : LOW);
  analogWrite(PWM_4, abs(pwm));
  pwm_out[1] = pwm;
}

void stopAll() { driveJ3(0); driveJ4(0); }

void publishStatus() {
  st3.joint = "J3"; st3.setpoint = v_cmd[0]; st3.pulse_count = 0; st3.error = 0;
  st3.output = pwm_out[0]; st3.control_loop = LOOP_HZ; st3.IsDone = (pwm_out[0] == 0);
  st4.joint = "J4"; st4.setpoint = v_cmd[1]; st4.pulse_count = 0; st4.error = 0;
  st4.output = pwm_out[1]; st4.control_loop = LOOP_HZ; st4.IsDone = (pwm_out[1] == 0);
  pub3.publish(&st3);
  pub4.publish(&st4);
}

void setup() {
  pinMode(ENABLE_3, OUTPUT); digitalWrite(ENABLE_3, HIGH);
  pinMode(ENABLE_4, OUTPUT); digitalWrite(ENABLE_4, HIGH);
  pinMode(HBRIDGE_3A, OUTPUT); pinMode(HBRIDGE_3B, OUTPUT);
  pinMode(HBRIDGE_4A, OUTPUT); pinMode(HBRIDGE_4B, OUTPUT);
  pinMode(PWM_3, OUTPUT); pinMode(PWM_4, OUTPUT);
  pinMode(LS_3A, INPUT); pinMode(LS_3B, INPUT); pinMode(LS_4A, INPUT); pinMode(LS_4B, INPUT);
  pinMode(RELAY, OUTPUT); digitalWrite(RELAY, HIGH); brake_locked = true;
  stopAll();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub3);
  nh.advertise(pub4);
  nh.loginfo("Joints34_vel: malha aberta, graus/s em set_3/set_4, sem encoder");
}

void loop() {
  static unsigned long t_status = 0;
  nh.spinOnce();
  bool wd = (millis() - t_last_msg) > WATCHDOG_MS;
  if (estop || wd || t_last_msg == 0) {
    stopAll();
  } else {
    driveJ3(pwmFor(DIR_SIGN_3 * v_cmd[0], PWM_MIN_3, PWM_MAX_3, K_PWM_3));
    driveJ4(pwmFor(DIR_SIGN_4 * v_cmd[1], PWM_MIN_4, PWM_MAX_4, K_PWM_4));
  }
  if (millis() - t_status >= 50) {
    t_status = millis();
    publishStatus();
  }
  delay(1000 / LOOP_HZ);
}
