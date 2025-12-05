#include <Ps3Controller.h>
#include <Arduino.h>
#include <cmath>

// -------------------------------
// CONFIGURAÇÃO PWM (LEDC)
// -------------------------------
#define PWM_FREQ 5000
#define PWM_RES  8
#define M1_CH    0
#define M2_CH    1
#define M3_CH    2

// -------------------------------
// ESTRUTURAS
// -------------------------------
struct velRodas
{
  float w1;
  float w2;
  float w3;
};

// -------------------------------
// PINOS DOS MOTORES
// -------------------------------
#define M1_IN1 12
#define M1_IN2 14
#define M1_PWM 13
#define M1_C1 35
#define M1_C2 34

#define M2_IN1 27
#define M2_IN2 26
#define M2_PWM 25

#define M3_IN1 19
#define M3_IN2 18
#define M3_PWM 21

// -------------------------------
// VARIÁVEIS FÍSICAS
// -------------------------------
const float centro = 0.133f;
const float raio = 0.033f;

const float vMax = 0.30f; // linear
const float rMax = 1.0f;  // rotação

const float CPR = 468.0f;

// -------------------------------
// VARIÁVEIS DE LEITURA DO ENCODER
// -------------------------------
volatile long posMotor1 = 0;
volatile uint8_t lastStateM1 = 0;

unsigned long ultimaVez = 0;
long ultimaPos1 = 0;

// -------------------------------
// FUNÇÕES DE ENCODER
// -------------------------------
void IRAM_ATTR atualizaPosicaoMotor() {
  uint8_t state = (digitalRead(M1_C1) << 1) | digitalRead(M1_C2);

  int8_t lookup[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
  };

  posMotor1 += lookup[(lastStateM1 << 2) | state];
  lastStateM1 = state;
}

// -------------------------------
// DEADZONE JOYSTICK
// -------------------------------
float deadzone(float value, float zone = 0.20f)
{
  if (fabs(value) < zone)
    return 0.0f;
  return value;
}

// -------------------------------
// CINEMÁTICA
// -------------------------------
velRodas conversao(float lx, float ly, float rx)
{
  // Normaliza entradas do PS3 (-128 a 127) para unidades físicas
  float vx = (ly / 128.0f) * vMax;
  float vy = (lx / 128.0f) * vMax;
  float w = (rx / 128.0f) * rMax;

  vx = deadzone(vx, 0.05f);
  vy = deadzone(vy, 0.05f);
  w  = deadzone(w, 0.05f);

  velRodas vel;
  const float L = centro;
  const float r = raio;

  float sqrt3_sobre_2 = sqrt(3.0f) / 2.0f;
  float meio = 0.5f;

  vel.w1 = (-sqrt3_sobre_2 / r) * vx + (meio / r) * vy + (L / r) * w;
  vel.w2 = (0.0f) * vx + (-1.0f / r) * vy + (L / r) * w;
  vel.w3 = (sqrt3_sobre_2 / r) * vx + (meio / r) * vy + (L / r) * w;

  return vel;
}

// -------------------------------
// FUNÇÃO DE CONTROLE DE MOTORES (MODIFICADA)
// -------------------------------
// Agora recebe o CANAL PWM (pwmChannel) em vez do pino
void setMotor(int IN1, int IN2, int pwmChannel, float velocidade)
{
  // Converte velocidade física para Duty Cycle (0-255)
  // Nota: Ajuste o fator multiplicador se sua velocidade sair muito baixa ou alta
  int pwmValue = int(fabs(velocidade) * 255); 
  
  if (pwmValue > 255) pwmValue = 255;

  if (velocidade > 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (velocidade < 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // Substitui analogWrite por ledcWrite
  ledcWrite(pwmChannel, pwmValue);
}

// -------------------------------
// SETUP
// -------------------------------
void setup()
{
  Serial.begin(115200);
  Ps3.begin("01:02:03:04:05:06");

  // --- Configuração Motor 1 ---
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  // Configura PWM Canal 0
  ledcSetup(M1_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1_PWM, M1_CH);
  
  // Encoder Motor 1
  pinMode(M1_C1, INPUT_PULLUP);
  pinMode(M1_C2, INPUT_PULLUP);
  lastStateM1 = (digitalRead(M1_C1) << 1) | digitalRead(M1_C2);
  attachInterrupt(digitalPinToInterrupt(M1_C1), atualizaPosicaoMotor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_C2), atualizaPosicaoMotor, CHANGE);

  // --- Configuração Motor 2 ---
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  // Configura PWM Canal 1
  ledcSetup(M2_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(M2_PWM, M2_CH);

  // --- Configuração Motor 3 ---
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  // Configura PWM Canal 2
  ledcSetup(M3_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(M3_PWM, M3_CH);
}

// -------------------------------
// LOOP
// -------------------------------
void loop()
{
  if (!Ps3.isConnected()) return;

  // Lendo analógicos (range -128 a 127)
  float lx = Ps3.data.analog.stick.lx;
  float ly = Ps3.data.analog.stick.ly;
  float rx = Ps3.data.analog.stick.rx;

  // Calcula cinemática
  velRodas v = conversao(lx, ly, rx);

  // Envia para os motores usando os Canais PWM definidos
  setMotor(M1_IN1, M1_IN2, M1_CH, v.w1);
  setMotor(M2_IN1, M2_IN2, M2_CH, v.w2);
  setMotor(M3_IN1, M3_IN2, M3_CH, v.w3);

  // --- Cálculo de RPM Motor 1 ---
  unsigned long agora = millis();
  unsigned long dt = agora - ultimaVez;

  if(dt >= 1000) {
    noInterrupts();
    long pos = posMotor1;
    interrupts();

    long delta = pos - ultimaPos1;
    float voltas = (float)delta / CPR;
    float rpm = voltas * (60000.0f / (float)dt);

    Serial.print("RPM1: ");
    Serial.println(rpm, 1);

    ultimaPos1 = pos;
    ultimaVez = agora;
  }
}