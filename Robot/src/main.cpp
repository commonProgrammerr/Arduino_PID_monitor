/*
 * Referencias:
 *  - Biblioteca PID: https://playground.arduino.cc/Code/PIDLibrary/
 *  - Código PID: https://playground.arduino.cc/Code/PIDLibraryPonMExample/
 *  - Datasheet drive L9110_2: https://cdn.awsli.com.br/945/945993/arquivos/L9110_2_CHANNEL_MOTOR_DRIVER.pdf
 *  - Código sensor ultrassonico: https://github.com/gamegine/HCSR04-ultrasonic-sensor-lib/blob/master/src/HCSR04.cpp
 *  - Comunicação serial: 
 *
 *
*/


#include <Arduino.h>
#include "PID_v1.h"

#define TRIG_PIN 2
#define ECHO_PIN 3

// sginal bites 
#define READY     0x0f // '►' 
#define STOP      0x24 // '$'
#define HEADER    0x3c // '<' 
#define TAIL      0x3e // '>' 
#define SKIP      0x25 // '%' 
#define REQUEST   0x40 // '@'
#define RESPONSE  0x13 // '‼'

// request/response bites
#define READ          0x26 // '&' letura da distância, potencia dos motores A e B, respectivamente.
#define SET_DISTANCE  0x22 // '"' declara a distância "objetiva" 
#define SET_POWER     0x21 // '!' declara a potência desejada dos motores

#define OK            0x5f // '_' bite de confirmação
#define ERROR         0x3a // ':' bite de erro 
#define EQUAL         0x3d // '=' bite de atribuição
#define SPLITER       0x3b // ';'


// constantes
#define K 29.4 // 29.4 microseconds para o som percorrer um centímetro.
#define Kp 2
#define Ki 5
#define Kd 1

class Motor;
float mensure();
void request();
void response();

double setpoint, distance, motors_power;
bool is_pause = true;
Motor left_motor;
Motor right_motor;

PID myPID(&distance, &motors_power, &setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);


void setup() {

  Serial.begin(2000000)
  Serial.readBytesUntil(READY)
  left_motor = new Motor(10, 9);
  right_motor = new Motor(5, 6);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  setpoint = 2.0
  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(AUTOMATIC);
}


void loop() {
  distance = mensure();
  myPID.Compute();
  left_motor.move(motors_power)
  right_motor.move(motors_power)
}

class Motor {
  private:
    int PortaA;
    int PortaB;

  public: 
    Motor(int PortaA, int PortaB) {
      this->PortaA = PortaA;
      this->PortaB = PortaB;
      pinMode(this->PortaA, OUTPUT); 
      pinMode(this->PortaB, OUTPUT); 
    }

    void move(int pow) {
      if(pow > 0 && pow <= 255) {
        analogWrite(this->portaA, pow)
        analogWrite(this->portaB, 0)
      } else if (pow < 0 && pow >= -255) {
        analogWrite(this->portaA, 0)
        analogWrite(this->portaB, pow * -1)
      } else {
        analogWrite(this->portaA, 0)
        analogWrite(this->portaB, 0)
      }
    }
}

float mensure() {
    // Ao emitir um ultra som, o sensor gera mais de uma onda ultrassonica. Afim de evitar erros de leitura é ncessario esperar pelo menos 200 microsegundos.
    delayMicroseconds(200);// espera o final do ruino.
    
    // Emite ondas ultrassonicas por 10 microssegundos (10 * 1^-10000 segundos)
    digitalWrite(TRIG, HIGH); 
    delayMicroseconds(10); 
    digitalWrite(TRIG, LOW);
    
    // Espera a leitura da primeira onda e calcula o a distancia de acodo com o intervalo de tempo obitido.
    return pulseIn(ECHO, HIGH) /  MS_CM/ 2; // o tempo é dividido por 2 pois a onda vai até o objeto e volta, percorrendo o caminho 2 vezes.
}
