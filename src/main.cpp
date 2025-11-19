#include <Ps3Controller.h>
#include <Arduino.h>
#include <cmath>

// PINOS DOS MOTORES (exemplo)
#define M1_IN1 12
#define M1_IN2 14
#define M1_PWM 13

#define M2_IN1 27
#define M2_IN2 26
#define M2_PWM 25

#define M3_IN1 19
#define M3_IN2 18
#define M3_PWM 21

// variaveis fisicas

const float centro = 0.133f;
const float raio = 0.033f;

// velocidades máximas

const float vMax = 0.30f; // linear
const float rMax = 1.0f;  // rotação

struct velRodas
{
  float w1;
  float w2;
  float w3;
};

float deadzone(float value, float zone = 0.20f)
{
  if (fabs(value) < zone)
    return 0.0f;
  return value;
}

velRodas conversao(float lx, float ly, float rx)
{
  // Normalização do joystick para velocidades físicas
  float vx = (lx / 128.0f) * vMax;
  float vy = (-ly / 128.0f) * vMax;
  float w = (rx / 128.0f) * rMax;

  velRodas vel;

  const float L = centro;
  const float r = raio;

  vx = deadzone(vx, 0.05f); // 5% de deadzone
  vy = deadzone(vy, 0.05f);
  w = deadzone(w, 0.05f);

  // constantes da cinemática
  float sqrt3_sobre_2 = sqrt(3.0f) / 2.0f;
  float meio = 0.5f;

  // Fórmulas da inversão da matriz cinemática
  vel.w1 = (-sqrt3_sobre_2 / r) * vx + (meio / r) * vy + (L / r) * w;
  vel.w2 = (0.0f) * vx + (-1.0f / r) * vy + (L / r) * w;
  vel.w3 = (sqrt3_sobre_2 / r) * vx + (meio / r) * vy + (L / r) * w;

  return vel;
}

void setMotor(int IN1, int IN2, int PWM, float velocidade)
{
  int pwmValue = int(fabs(velocidade) * 255); // converte de 0-1 para 0-255
  if (pwmValue > 255)
    pwmValue = 255; // limite de PWM

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

  analogWrite(PWM, pwmValue);
}

void setup()
{
  Ps3.begin("01:02:03:04:05:06");

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
}

void loop()
{
  if (!ps3IsConnected())
  {
    return;
  }

  // Lê os valores dos joysticks
  float lx = Ps3.data.analog.stick.lx;
  float ly = Ps3.data.analog.stick.ly;
  float rx = Ps3.data.analog.stick.rx;

  // Converte para velocidades das rodas
  velRodas v = conversao(lx, ly, rx);

  // Envia para os motores
  setMotor(M1_IN1, M1_IN2, M1_PWM, v.w1);
  setMotor(M2_IN1, M2_IN2, M2_PWM, v.w2);
  setMotor(M3_IN1, M3_IN2, M3_PWM, v.w3);
}