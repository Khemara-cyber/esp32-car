
#include <Arduino.h>

// Motor A
const int ENA = 14;
const int IN1 = 27;
const int IN2 = 26;

// Motor B
const int IN3 = 25;
const int IN4 = 33;
const int ENB = 32;

// Motor speeds (0-255)
const int SPEED_A = 100;
const int SPEED_B = 100;

void goStraight()
{
  // Motor A forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, SPEED_A);

  // Motor B forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, SPEED_B);
}

void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  goStraight();
}

void loop()
{
  // Motors continue running forward
}
