////////////////////////////////////////////////////////////////////////////////
//  Joints12_vel — firmware de VELOCIDADE (malha aberta) para a placa Joints12
//  (J1 cintura, J2 ombro) do RV-M2 retrofitado do b166er.          2026-10-07
//
//  POR QUE EXISTE. O braço NÃO tem encoder funcional em nenhuma junta. O
//  firmware "operacional" de 2022 (Joints12.ino) faz PD sobre uma contagem
//  de pulsos que não chega — não serve. A arquitetura do projeto fecha a
//  malha no espaço da tarefa, pela T265 no efetuador (estimador + Fuzzy);
//  a placa só precisa fazer o que o firmware de ensaio já fazia no
//  motorGo(): sentido + PWM. Este firmware recebe, por junta, uma VELOCIDADE
//  pedida em graus/s e a executa em malha aberta, com:
//    · zona morta (abaixo de V_DEAD o motor não é acionado);
//    · PWM = PWM_MIN + K_PWM·|v|, saturado em PWM_MAX;
//    · freio do J2 solto enquanto há comando, travado BRAKE_LOCK_MS depois
//      de a velocidade ir a zero;
//    · fins de curso: parar o sentido que encosta no switch (ativo em HIGH);
//    · watchdog: sem mensagem por WATCHDOG_MS, tudo parado e freio travado;
//    · emergency_stop: idem, imediato.
//
//  MENSAGEM. movemaster_msg/setpoint, a mesma de sempre:
//    set_1 = velocidade do J1 (graus/s), set_2 = velocidade do J2 (graus/s),
//    set_3..set_5 ignorados aqui, set_GRIP ignorado aqui, emergency_stop,
//    GoHome ignorado (era o seletor do firmware de ensaio).
//  É exatamente o que o arm_joint_servo.py publica em /setpoints, e o que
//  o arm_openloop_sim.py consome na simulação (mesma convenção).
//
//  SINAL DO SENTIDO. DIR_SIGN_x = +1 se "CW" do motorGo corresponde ao
//  sentido positivo da junta no modelo (URDF/kinematics), −1 se não. Tem de
//  ser conferido na bancada, uma vez por junta: comandar +5 graus/s e ver se
//  a estimativa (T265) cresce. Sinal errado = o servo diverge.
//
//  STATUS. /status_1 e /status_2 (movemaster_msg/status): setpoint = v
//  pedida (graus/s), output = PWM aplicado (com sinal), pulse_count = 0
//  (não há encoder), IsDone = motor parado. Mantém o publicador de estado
//  e as ferramentas de bancada funcionando sem mudança.
//
//  Pinos: os mesmos do Joints12_test.ino (validados na bancada em 06/10/2026:
//  J1 girou nos dois sentidos com o código 1).
////////////////////////////////////////////////////////////////////////////////
#include <ros.h>
#include <movemaster_msg/setpoint.h>
#include <movemaster_msg/status.h>

// ---- pinos (= Joints12_test.ino) -------------------------------------------
#define HBRIDGE_1A   4     // J1 motor pin A
#define HBRIDGE_1B   9     // J1 motor pin B
#define PWM_1        6     // J1 PWM
#define ENABLE_1     A1    // J1 enable
#define PWM_2A       3     // J2 PWM A (BTS7960)
#define PWM_2B       2     // J2 PWM B (BTS7960)
#define ENABLE_2A    10    // J2 enable A
#define ENABLE_2B    11    // J2 enable B
#define LS_1A        34    // J1 fim de curso A (lado CW)  — ativo em HIGH
#define LS_1B        36    // J1 fim de curso B (lado CCW)
#define LS_2A        38    // J2 fim de curso A (lado CW)
#define LS_2B        40    // J2 fim de curso B (lado CCW)
#define RELAY        14    // freio do J2: HIGH = travado

// ---- parâmetros (ajustar na bancada) ---------------------------------------
#define DIR_SIGN_1    (+1)   // +1: CW = sentido positivo do J1 no modelo
#define DIR_SIGN_2    (+1)
#define V_DEAD        1.5f   // graus/s — abaixo disso não aciona
#define V_MAX         60.0f  // graus/s — saturação do pedido
#define PWM_MIN_1     60     // vence o atrito do redutor (medir)
#define PWM_MIN_2     80
#define PWM_MAX       230
#define K_PWM_1       2.8f   // PWM por grau/s (PWM_MIN + K·|v|); 60°/s -> ~228
#define K_PWM_2       2.5f
#define BRAKE_LOCK_MS 300    // trava o freio do J2 este tempo após v = 0
#define WATCHDOG_MS   500
#define LOOP_HZ       50

ros::NodeHandle nh;
movemaster_msg::status st1, st2;
ros::Publisher pub1("status_1", &st1);
ros::Publisher pub2("status_2", &st2);

float v_cmd[2] = {0.0f, 0.0f};     // graus/s pedidos (J1, J2)
int   pwm_out[2] = {0, 0};         // PWM aplicado com sinal
unsigned long t_last_msg = 0;
unsigned long t_j2_zero  = 0;      // instante em que v2 foi a zero
bool estop = false;
bool brake_locked = true;

void brake2(bool lock) {
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
  v_cmd[0] = constrain(m.set_1, -V_MAX, V_MAX);
  v_cmd[1] = constrain(m.set_2, -V_MAX, V_MAX);
}
ros::Subscriber<movemaster_msg::setpoint> sub("setpoints", &callback);

// v (graus/s, já com DIR_SIGN aplicado) -> PWM com sinal, respeitando zona morta
int pwmFor(float v, int pwm_min, float k) {
  float a = fabs(v);
  if (a < V_DEAD) return 0;
  int p = (int)(pwm_min + k * a);
  if (p > PWM_MAX) p = PWM_MAX;
  return (v > 0) ? p : -p;
}

void driveJ1(int pwm) {
  // fins de curso: não empurrar contra o switch que já está acionado
  if (pwm > 0 && digitalRead(LS_1A) == HIGH) pwm = 0;
  if (pwm < 0 && digitalRead(LS_1B) == HIGH) pwm = 0;
  digitalWrite(HBRIDGE_1A, pwm > 0 ? HIGH : LOW);
  digitalWrite(HBRIDGE_1B, pwm < 0 ? HIGH : LOW);
  analogWrite(PWM_1, abs(pwm));
  pwm_out[0] = pwm;
}

void driveJ2(int pwm) {
  if (pwm > 0 && digitalRead(LS_2A) == HIGH) pwm = 0;
  if (pwm < 0 && digitalRead(LS_2B) == HIGH) pwm = 0;
  if (pwm != 0) {
    brake2(false);                    // solta o freio para mover
    t_j2_zero = 0;
  } else {
    if (t_j2_zero == 0) t_j2_zero = millis();
    if (millis() - t_j2_zero >= BRAKE_LOCK_MS) brake2(true);
  }
  // BTS7960: um PWM por sentido (igual ao motorGo do Joints12_test)
  analogWrite(PWM_2B, pwm > 0 ? pwm : 0);    // CW
  analogWrite(PWM_2A, pwm < 0 ? -pwm : 0);   // CCW
  pwm_out[1] = pwm;
}

void stopAll() {
  driveJ1(0);
  driveJ2(0);
}

void publishStatus() {
  st1.joint = "J1"; st1.setpoint = v_cmd[0]; st1.pulse_count = 0; st1.error = 0;
  st1.output = pwm_out[0]; st1.control_loop = LOOP_HZ; st1.IsDone = (pwm_out[0] == 0);
  st2.joint = "J2"; st2.setpoint = v_cmd[1]; st2.pulse_count = 0; st2.error = 0;
  st2.output = pwm_out[1]; st2.control_loop = LOOP_HZ; st2.IsDone = (pwm_out[1] == 0);
  pub1.publish(&st1);
  pub2.publish(&st2);
}

void setup() {
  pinMode(ENABLE_1, OUTPUT);  digitalWrite(ENABLE_1, HIGH);
  pinMode(ENABLE_2A, OUTPUT); digitalWrite(ENABLE_2A, HIGH);
  pinMode(ENABLE_2B, OUTPUT); digitalWrite(ENABLE_2B, HIGH);
  pinMode(HBRIDGE_1A, OUTPUT); pinMode(HBRIDGE_1B, OUTPUT);
  pinMode(PWM_1, OUTPUT); pinMode(PWM_2A, OUTPUT); pinMode(PWM_2B, OUTPUT);
  pinMode(LS_1A, INPUT); pinMode(LS_1B, INPUT); pinMode(LS_2A, INPUT); pinMode(LS_2B, INPUT);
  pinMode(RELAY, OUTPUT); digitalWrite(RELAY, HIGH); brake_locked = true;
  stopAll();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub1);
  nh.advertise(pub2);
  nh.loginfo("Joints12_vel: malha aberta, graus/s em set_1/set_2, sem encoder");
}

void loop() {
  static unsigned long t_status = 0;
  nh.spinOnce();
  bool wd = (millis() - t_last_msg) > WATCHDOG_MS;
  if (estop || wd || t_last_msg == 0) {
    stopAll();
  } else {
    driveJ1(pwmFor(DIR_SIGN_1 * v_cmd[0], PWM_MIN_1, K_PWM_1));
    driveJ2(pwmFor(DIR_SIGN_2 * v_cmd[1], PWM_MIN_2, K_PWM_2));
  }
  if (millis() - t_status >= 50) {   // status a 20 Hz
    t_status = millis();
    publishStatus();
  }
  delay(1000 / LOOP_HZ);
}
