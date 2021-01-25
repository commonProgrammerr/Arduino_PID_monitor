/*
 * Referencias:
 *  - Biblioteca PID: https://playground.arduino.cc/Code/PIDLibrary/
 *  - Código PID: https://playground.arduino.cc/Code/PIDLibraryPonMExample/
 *  - Datasheet drive L9110_2: https://cdn.awsli.com.br/945/945993/arquivos/L9110_2_CHANNEL_MOTOR_DRIVER.pdf
 *  - Código sensor ultrassonico: https://github.com/gamegine/HCSR04-ultrasonic-sensor-lib/blob/master/src/HCSR04.cpp
 *
*/

#include <Arduino.h>
#include "../include/Arduino-PID-Library/PID_v1.cpp" // um bug faz com que o compilador não encontre os arquivos de forma correta, para contornar isso fizemos o include no aquivo .cpp


// pinos
#define TRIG_PIN 2
#define ECHO_PIN 3

// sginal bytes §
#define READY   '#'   // 0x23
#define HEADER  '<'   // 0x3c
#define TAIL    '>'   // 0x3e
#define RECIVE  '@'   // 0x40
#define SEND    '%'   // 0x25  

// RECIVE/SEND bytes
#define SET_DISTANCE  '!' // 0x21  declara a distância "objetiva" 
#define REQUEST_DATA  '_' // 0x5f  byte de requisição
#define SPLITER       ';' // 0x3b  byte de separação de valores

// constantes
#define K 29.4 // 29.4 microseconds para o som percorrer um centímetro.
#define K_drive 1.07 // contante para equilibra a diferença de potencia entre os motores esquedo e direito
#define Kp 2 * 10 // 23.0
#define Ki 5 * 10 // 0.0
#define Kd 1 * 10 // 0.0

class Motor {
  private:
    int portaA;
    int portaB;

  public: 
    Motor(int portaA, int portaB);
    void move(double pow);
};

float mensure();
void awai_request();
void send_data();

double setpoint, distance, motors_power = 0.0;
Motor left_motor(5, 6);
Motor right_motor(10, 9);

PID pid(&distance, &motors_power, &setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);

void setup() {

  Serial.begin(9600);
  Serial.setTimeout(2);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  setpoint = 8.0;
  pid.SetOutputLimits(-175, 175);
  pid.SetMode(AUTOMATIC);
}

void loop() {
  distance = mensure();
  pid.Compute();
  left_motor.move(-motors_power * K_drive);
  right_motor.move(-motors_power);
  awai_request();
}

void awai_request() {
  Serial.readStringUntil(RECIVE);
  Serial.readStringUntil(HEADER);
  String data = Serial.readStringUntil(TAIL);

  bool is_setup_distance = (data.indexOf(SET_DISTANCE) >= 0);
  bool is_data_require = (data.indexOf(REQUEST_DATA) >= 0);
  
  send_data();
  
  if(is_setup_distance) {
    data.remove(READY);
    data.remove(HEADER);
    data.remove(TAIL);
    data.remove(RECIVE);
    data.remove(SEND);
    data.remove(SET_DISTANCE);
    data.remove(REQUEST_DATA);
    data.remove(SPLITER);
    setpoint = data.toDouble();
  }

}

void send_data() {
  Serial.print(SEND);
  Serial.print(HEADER);
  Serial.print(distance);
  Serial.print(SPLITER);
  Serial.print(motors_power);
  Serial.print(TAIL);
}

float mensure() {
    
    // Ao emitir um ultra som, o sensor gera mais de uma onda ultrassonica. Afim de evitar erros de leitura é ncessario esperar pelo menos 2 microssegundos.
    delayMicroseconds(20);// espera o final do ruino.

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

    analogWrite(this->portaA, 0);
    analogWrite(this->portaB, pow);
  
  } else if (pow < 0.0 && pow >= -255.0) {
  
    analogWrite(this->portaB, 0);
    analogWrite(this->portaA, pow * -1);

  } else {
  
    analogWrite(this->portaA, 0);
    analogWrite(this->portaB, 0);
  }
}