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

// velocidades maximas

const float vMax = 0.30f; // linear
const float rMax = 1.0f;  // rotação

// velocidade motor

const float CPR_MOTOR = 468.0f; // "counts per revolution" obtido pela formula relacaoDaCaixaDeReducao x pulsosPorVoltaDoDisco

// leitura de cada encoder

struct Encoder
{
  volatile long position;   // gravam o movimento do encoder
  long lastPosition;        // gravam o momento do ultimo movimento do encoder
  unsigned long lastMillis; // variavel auxilixar para cada millis
};

// separa a leitura de cada encoder
Encoder encoders[3];

// le os encoders

void IRAM_ATTR encoder1()
{
  if (digitalRead(M1_C1) == digitalRead(M1_C2))
    encoders[0].position++;
  else
    encoders[0].position--;
}
void IRAM_ATTR encoder2()
{
  if (digitalRead(M2_C1) == digitalRead(M2_C2))
    encoders[1].position++;
  else
    encoders[1].position--;
}
void IRAM_ATTR encoder3()
{
  if (digitalRead(M3_C1) == digitalRead(M3_C2))
    encoders[2].position++;
  else
    encoders[2].position--;
}

float deadzone(float value, float zone = 0.20f)
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

  vx = deadzone(vx, 0.05f); // 5% de deadzone
  vy = deadzone(vy, 0.05f);
  w = deadzone(w, 0.05f);

  // constantes da cinematica
  float sqrt3_sobre_2 = sqrt(3.0f) / 2.0f;
  float meio = 0.5f;

  // formulas da inversão da matriz cinematica
  vel.w1 = (-sqrt3_sobre_2 / r) * vx + (meio / r) * vy + (L / r) * w;
  vel.w2 = (0.0f) * vx + (-1.0f / r) * vy + (L / r) * w;
  vel.w3 = (sqrt3_sobre_2 / r) * vx + (meio / r) * vy + (L / r) * w;

  return vel;
}

void setMotor(int IN1, int IN2, int pwmPin, float velocidade)
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

  analogWrite(pwmPin, pwmValue);
}

float getRPM(int whichEncoder)
{
  unsigned long now = millis();
  unsigned long deltaTime = now - encoders[whichEncoder].lastMillis;

  noInterrupts();
  long pos = encoders[whichEncoder].position;
  interrupts();

  long delta = pos - encoders[whichEncoder].lastPosition;

  if (deltaTime == 0)
    return 0;

  float rpm = ((float)delta / CPR_MOTOR) * (60000.0f / (float)deltaTime);

  encoders[whichEncoder].lastPosition = pos;
  encoders[whichEncoder].lastMillis = now;
  return rpm;
}

void setup()
{
  Ps3.begin("01:02:03:04:05:06");
  Serial.begin(115200);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_C1, INPUT_PULLUP);
  pinMode(M1_C2, INPUT_PULLUP);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_C1, INPUT_PULLUP);
  pinMode(M2_C2, INPUT_PULLUP);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(M3_C1, INPUT_PULLUP);
  pinMode(M3_C2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_C1), encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_C1), encoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(M3_C1), encoder3, RISING);

  for (int i = 0; i < 3; i++)
  {
    encoders[i].position = 0;
    encoders[i].lastPosition = 0;
    encoders[i].lastMillis = millis();
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