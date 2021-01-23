/*
 * Referencias:
 *  - Biblioteca PID: https://playground.arduino.cc/Code/PIDLibrary/
 *  - Código PID: https://playground.arduino.cc/Code/PIDLibraryPonMExample/
 *  - Datasheet drive L9110_2: https://cdn.awsli.com.br/945/945993/arquivos/L9110_2_CHANNEL_MOTOR_DRIVER.pdf
 *  - Código sensor ultrassonico: https://github.com/gamegine/HCSR04-ultrasonic-sensor-lib/blob/master/src/HCSR04.cpp
 *
*/

#include <Arduino.h>
#include "../include/Arduino-PID-Library/PID_v1.h"


// pinos
#define TRIG_PIN 2
#define ECHO_PIN 3

// sginal bytes 
#define READY   '►' // 0x0f
#define STOP    '$' // 0x24
#define HEADER  '<' // 0x3c
#define TAIL    '>' // 0x3e
#define RECIVE  '@' // 0x40
#define SEND    '‼' // 0x13

// RECIVE/SEND bytes
#define SET_DISTANCE  '!' // 0x21  declara a distância "objetiva" 
#define OK            '_' // 0x5f  byte de confirmação
#define ERROR         ':' // 0x3a  byte de erro 
#define SPLITER       ';' // 0x3b  byte de separação de valores


// constantes
#define K 29.4 // 29.4 microseconds para o som percorrer um centímetro.
#define Kp 2
#define Ki 5
#define Kd 1

class Motor {
  private:
    int portaA;
    int portaB;

  public: 
    Motor(int portaA, int portaB);
    void move(double pow);
};

float mensure();
void send(String info);
void read();



double setpoint, distance, motors_power;
bool is_pause = true;
Motor left_motor(10, 9);
Motor right_motor(5, 6);

PID pid(&distance, &motors_power, &setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);

void setup() {

  Serial.begin(2000000);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  setpoint = 4.0;
  pid.SetOutputLimits(-255, 255);
  pid.SetMode(AUTOMATIC);
}

void loop() {
  read();
  if(is_pause) {
    if(READY == Serial.read()) is_pause = false;
    else return;
  } 
  
  distance = mensure();
  
  pid.Compute();
  
  String output = "";
  output = output.concat(String(distance));
  output = output.concat(String(SPLITER));
  output = output.concat(String(motors_power));
  
  send(output);
  output = "";
  
  left_motor.move(motors_power);
  right_motor.move(motors_power); 
}

void read() {
  Serial.readStringUntil(RECIVE);
  Serial.readStringUntil(HEADER);
  String data = Serial.readStringUntil(TAIL);

  bool erro = (data.indexOf(ERROR) >= 0);
  bool stop = (data.indexOf(STOP) >= 0);
  bool set_distance = (data.indexOf(SET_DISTANCE) >= 0);

  if(erro || stop) is_pause = true;
    
  if(set_distance) {
    data.remove(READY);
    data.remove(STOP);
    data.remove(HEADER);
    data.remove(TAIL);
    data.remove(RECIVE);
    data.remove(SEND);
    data.remove(SET_DISTANCE);
    data.remove(OK);
    data.remove(ERROR);
    data.remove(SPLITER);
    setpoint = data.toDouble();
  }

}

void send(String info) {
  Serial.print(SEND);
  Serial.print(HEADER);
  Serial.print(info);
  Serial.print(TAIL);
}

float mensure() {
    
    // Ao emitir um ultra som, o sensor gera mais de uma onda ultrassonica. Afim de evitar erros de leitura é ncessario esperar pelo menos 2 microssegundos.
    delayMicroseconds(2);// espera o final do ruino.

    // Emite ondas ultrassonicas por 10 microssegundos (10 * 1^-10000 segundos)
    digitalWrite(TRIG_PIN, HIGH); 
    delayMicroseconds(10); 
    digitalWrite(TRIG_PIN, LOW);
    
    // Espera a leitura da primeira onda e calcula o a distancia de acodo com o intervalo de tempo obitido.
    return pulseIn(ECHO_PIN, HIGH) /  K/ 2; // o tempo é dividido por 2 pois a onda vai até o objeto e volta, percorrendo o caminho 2 vezes.
}

Motor::Motor(int portaA, int portaB) {
  this->portaA = portaA;
  this->portaB = portaB;
  pinMode(this->portaA, OUTPUT); 
  pinMode(this->portaB, OUTPUT); 
}

void Motor::move(double pow) {

  if(pow > 0.0 && pow <= 255.0) {

    analogWrite(this->portaA, pow);
    analogWrite(this->portaB, 0);
  
  } else if (pow < 0.0 && pow >= -255) {
  
    analogWrite(this->portaA, 0);
    analogWrite(this->portaB, pow * -1);

  } else {
  
    analogWrite(this->portaA, 0);
    analogWrite(this->portaB, 0);
  }
}