#include <ESP32Encoder.h>
#include <Ps3Controller.h>
#include <Arduino.h>
#include <cmath>


// define as diferentes rodas
struct velRodas
{
  float w1;
  float w2;
  float w3;
};

// PINOS DOS MOTORES
#define M1_IN1 12
#define M1_IN2 14
#define M1_PWM 13
#define M1_C1 4
#define M1_C2 5

#define M2_IN1 27
#define M2_IN2 26
#define M2_PWM 25
#define M2_C1 16
#define M2_C2 2

#define M3_IN1 19
#define M3_IN2 18
#define M3_PWM 21
#define M3_C1 15
#define M3_C2 17

// variaveis fisicas
const float centro = 0.133f;
const float raio = 0.033f;

// ⚠️ CORREÇÃO 1: Ajusta vMax para atingir o RPM máximo do motor (~280 RPM)
const float W_MAX_MOTOR_RAD_S = 29.32f; // ~280 RPM
const float vMax = W_MAX_MOTOR_RAD_S * raio; // Novo vMax = 0.967 m/s
const float rMax = 1.0f;  // rotação

// velocidade motor
// ⚠️ CORREÇÃO 2: Novo valor calculado (643.0f) para que 280 RPM sejam lidos como 280
const float CPR_MOTOR = 643.0f; 

// --- Configuração da Biblioteca ESP32Encoder ---

ESP32Encoder encoder1Obj;
ESP32Encoder encoder2Obj;
ESP32Encoder encoder3Obj;

ESP32Encoder* encodersArray[] = {&encoder1Obj, &encoder2Obj, &encoder3Obj};

long lastPosition[3] = {0, 0, 0};
unsigned long lastMillis[3] = {0, 0, 0};

// ----------------------------------------------

float deadzone(float value, float zone = 0.05f) // Reduzido para 5%
{
  if (fabs(value) < zone)
    return 0.0f;
  return value;
}

velRodas conversao(float lx, float ly, float rx)
{
  // normalização do joystick para velocidades físicas
  float vx = (ly / 128.0f) * vMax;
  float vy = (lx / 128.0f) * vMax;
  float w = (rx / 128.0f) * rMax;

  velRodas vel;

  const float L = centro;
  const float r = raio;

  vx = deadzone(vx);
  vy = deadzone(vy);
  w = deadzone(w);

  // constantes da cinematica
  float sqrt3_sobre_2 = sqrt(3.0f) / 2.0f;
  float meio = 0.5f;

  // formulas da inversão da matriz cinematica
  vel.w1 = (-sqrt3_sobre_2 / r) * vx + (meio / r) * vy + (L / r) * w;
  vel.w2 = (0.0f) * vx + (-1.0f / r) * vy + (L / r) * w;
  vel.w3 = (sqrt3_sobre_2 / r) * vx + (meio / r) * vy + (L / r) * w;

  return vel;
}

void setMotor(int IN1, int IN2, int pwmPin, float velocidade_rad_s)
{
  // ⚠️ CORREÇÃO 3: Escala o rad/s para o PWM. 
  // Agora W_MAX_MOTOR_RAD_S (29.32 rad/s) corresponde a 100% (255) do PWM.
  float porcentagem = fabs(velocidade_rad_s) / W_MAX_MOTOR_RAD_S;

  if (porcentagem > 1.0f)
    porcentagem = 1.0f; // Limita a 100%

  int pwmValue = int(porcentagem * 255); 

  if (velocidade_rad_s > 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (velocidade_rad_s < 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    pwmValue = 0;
  }

  analogWrite(pwmPin, pwmValue);
}

float getRPM(int whichEncoder)
{
  unsigned long now = millis();
  unsigned long deltaTime = now - lastMillis[whichEncoder];

  // Leitura da posição diretamente do objeto da biblioteca
  long pos = encodersArray[whichEncoder]->getCount(); 

  long delta = pos - lastPosition[whichEncoder];

  if (deltaTime == 0)
    return 0;

  // A fórmula usa CPR_MOTOR=643.0f
  float rpm = ((float)delta / CPR_MOTOR) * (60000.0f / (float)deltaTime);

  lastPosition[whichEncoder] = pos;
  lastMillis[whichEncoder] = now;
  return rpm;
}


// --- SETUP CORRIGIDO ---
void setup()
{
  Ps3.begin("01:02:03:04:05:06");
  Serial.begin(115200);
  
  // Conecta os pinos dos encoders (PCNT)
  encoder1Obj.attachFullQuad(M1_C1, M1_C2);
  encoder2Obj.attachFullQuad(M2_C1, M2_C2);
  encoder3Obj.attachFullQuad(M3_C1, M3_C2);

  // Zera os contadores
  encoder1Obj.setCount(0);
  encoder2Obj.setCount(0);
  encoder3Obj.setCount(0);
  
  // Configuração dos Pinos de Controle (Mantido)
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M3_PWM, OUTPUT);

  // Inicializa as variáveis de tempo e posição para o cálculo do RPM
  for (int i = 0; i < 3; i++)
  {
    lastPosition[i] = 0;
    lastMillis[i] = millis();
  }
}

void loop()
{
  if (!Ps3.isConnected())
    return;

  // le os valores dos joysticks
  float lx = Ps3.data.analog.stick.lx;
  float ly = Ps3.data.analog.stick.ly;
  float rx = Ps3.data.analog.stick.rx;

  // converte para velocidades das rodas
  velRodas v = conversao(lx, ly, rx);

  // envia para os motores
  setMotor(M1_IN1, M1_IN2, M1_PWM, v.w1);
  setMotor(M2_IN1, M2_IN2, M2_PWM, v.w2);
  setMotor(M3_IN1, M3_IN2, M3_PWM, v.w3);

  // exibe o rpm
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200)
  {
    Serial.print("RPM1: ");
    Serial.print(getRPM(0), 1);
    Serial.print(" | RPM2: ");
    Serial.print(getRPM(1), 1);
    Serial.print(" | RPM3: ");
    Serial.println(getRPM(2), 1);

    lastPrint = millis();
  }
}