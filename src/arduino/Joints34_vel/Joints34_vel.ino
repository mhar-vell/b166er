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
//  Atenção ao J4: o URDF limita o esforço a 4,2 N·m e a simulação (RELATORIO11)
//  mostrou que o punho cede no libera. PWM_MAX_4 era 180 por isso; a bancada
//  (17 Nov 2026) mostrou que a ponta só sobe com 155–176, então o teto foi a
//  230 para sobrar margem acima do limiar de subida.
//
//  PWM mínimo POR SENTIDO (18 Nov 2026): J3 e J4 carregam peso, e o limiar de
//  subida é bem maior que o de descida. _POS vale para PWM > 0 e _NEG para
//  PWM < 0, já depois do DIR_SIGN.
////////////////////////////////////////////////////////////////////////////////
#include <ros.h>
#include <movemaster_msg/setpoint.h>
#include <movemaster_msg/status.h>
#include <std_msgs/Int8MultiArray.h>

// ---- pinos (= Joints34_test.ino) -------------------------------------------
#define HBRIDGE_3A   4
#define HBRIDGE_3B   9
#define HBRIDGE_4A   7
#define HBRIDGE_4B   8
#define PWM_3        6
#define PWM_4        5
#define ENABLE_3     A1
#define ENABLE_4     A0
// LS_3A/LS_3B TROCADOS (17 Nov 2026, bancada): a chave que fecha com o cotovelo
// dobrado (pino 36) é a do sentido em que o motor gira com PWM > 0. Com 34/36
// na ordem antiga, o firmware bloqueava justamente o sentido de sair do batente.
#define LS_3A        36    // ativo em HIGH; A = lado do PWM > 0, B = lado do PWM < 0
#define LS_3B        34
#define LS_4A        40    // J4: fim de curso da ponta para BAIXO (lado do PWM > 0) — 18 Nov 2026, trocado na bancada
#define LS_4B        38    // J4: fim de curso da ponta para CIMA (lado do PWM < 0, positivo do modelo)
#define RELAY        14    // freio do J3: HIGH = travado

// ---- parâmetros (ajustar na bancada) ---------------------------------------
#define DIR_SIGN_3    (-1)   // 17 Nov 2026: PWM > 0 dobra o cotovelo; positivo do manual/modelo = antebraço sobe
#define DIR_SIGN_4    (-1)   // 17 Nov 2026: PWM > 0 baixa a ponta; positivo do manual/modelo = ponta sobe
#define V_DEAD        1.5f
#define V_MAX         60.0f
#define PWM_MIN_3_POS 70    // J3 descendo (PWM > 0): não medido — o pino 36 lê sempre acionado e bloqueia
#define PWM_MIN_3_NEG 175   // J3 subindo (PWM < 0): limiar entre 165 e 179, igual com bateria fraca
                            // (18 Nov: 159 não, 179 → 4 mm/1,5 s) e boa (19 Nov: 165 não, 179 → 8 mm,
                            // 199 → 24 mm). A bateria muda a velocidade, não o limiar.
#define PWM_MIN_4_POS 80    // J4 ponta descendo (PWM > 0): 17 Nov 2026, 60 pouco, 80 sim
#define PWM_MIN_4_NEG 160   // J4 ponta subindo (PWM < 0): 17 Nov 2026, 140 não, 155 pouco, 176 sim
#define PWM_MAX_3     230
#define PWM_MAX_4     230
#define K_PWM_3       2.6f
#define K_PWM_4       2.1f
#define BRAKE_HOLD_MS 150   // J3: ao zerar o comando, trava o freio JÁ e mantém o último PWM
                            // por este tempo, até o freio pegar; só então corta (17 Nov 2026:
                            // com 300 ms de freio aberto sem força, o antebraço caía).
#define WATCHDOG_MS   500
#define LOOP_HZ       50

ros::NodeHandle nh;
movemaster_msg::status st3, st4;
ros::Publisher pub3("status_3", &st3);
ros::Publisher pub4("status_4", &st4);
// FINS DE CURSO NO ROS (17 Out 2026): −1 = switch do lado NEGATIVO da junta no
// modelo (B se DIR_SIGN=+1), +1 = lado positivo (A), 0 = livre. Publicado a 20 Hz
// junto do status; o nó arm_switch_bridge.py junta as três placas em
// /b166er/arm_limit_switch (JointState), que o HOMING e o estimador consomem.
std_msgs::Int8MultiArray sw_msg;
int8_t sw_data[2];
ros::Publisher pub_sw("arm_switch_34", &sw_msg);

float v_cmd[2] = {0.0f, 0.0f};     // J3, J4
int   pwm_out[2] = {0, 0};
unsigned long t_last_msg = 0;
unsigned long t_j3_zero  = 0;
int last_pwm3 = 0;
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

int pwmFor(float v, int pwm_min_pos, int pwm_min_neg, int pwm_max, float k) {
  float a = fabs(v);
  if (a < V_DEAD) return 0;
  int p = (int)(((v > 0) ? pwm_min_pos : pwm_min_neg) + k * a);
  if (p > pwm_max) p = pwm_max;
  return (v > 0) ? p : -p;
}

bool blockedJ3(int pwm) {
  return (pwm > 0 && digitalRead(LS_3A) == HIGH) || (pwm < 0 && digitalRead(LS_3B) == HIGH);
}

void driveJ3(int pwm) {
  if (blockedJ3(pwm)) pwm = 0;
  if (blockedJ3(last_pwm3)) last_pwm3 = 0;  // no fim de curso, corta sem a sobreposição
  if (pwm != 0) {
    brake3(false);
    t_j3_zero = 0;
    last_pwm3 = pwm;
  } else {
    brake3(true);                          // trava o freio primeiro...
    if (last_pwm3 != 0) {
      if (t_j3_zero == 0) t_j3_zero = millis();
      if (millis() - t_j3_zero < BRAKE_HOLD_MS) pwm = last_pwm3;  // ...com o motor ainda segurando
      else last_pwm3 = 0;
    }
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
  sw_data[0] = (int8_t)(DIR_SIGN_3 * ((digitalRead(LS_3A) == HIGH) ? 1 : ((digitalRead(LS_3B) == HIGH) ? -1 : 0)));
  sw_data[1] = (int8_t)(DIR_SIGN_4 * ((digitalRead(LS_4A) == HIGH) ? 1 : ((digitalRead(LS_4B) == HIGH) ? -1 : 0)));
  sw_msg.data = sw_data; sw_msg.data_length = 2;
  pub_sw.publish(&sw_msg);
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
  nh.advertise(pub_sw);
  nh.advertise(pub3);
  nh.advertise(pub4);
  nh.loginfo("Joints34_vel: malha aberta, graus/s em set_3/set_4, sem encoder");
}

void loop() {
  static unsigned long t_status = 0;
  nh.spinOnce();
  bool wd = (millis() - t_last_msg) > WATCHDOG_MS;
  if (estop || wd || t_last_msg == 0) {
    if (estop) last_pwm3 = 0;             // emergência corta o PWM na hora
    stopAll();
  } else {
    driveJ3(pwmFor(DIR_SIGN_3 * v_cmd[0], PWM_MIN_3_POS, PWM_MIN_3_NEG, PWM_MAX_3, K_PWM_3));
    driveJ4(pwmFor(DIR_SIGN_4 * v_cmd[1], PWM_MIN_4_POS, PWM_MIN_4_NEG, PWM_MAX_4, K_PWM_4));
  }
  if (millis() - t_status >= 50) {
    t_status = millis();
    publishStatus();
  }
  delay(1000 / LOOP_HZ);
}
