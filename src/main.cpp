/**
 * @file main.cpp
 * @brief Main - Template
 * @author David Ortiz & Pablo Vargas
 * @date 04/08/2024
 */

#include <Arduino.h>
#include "../include/motor.h"
#include "../include/sensor.h"

#define BAUD_RATE 115200
#define ENABLE_MOTOR_A 13
#define IN1_MOTOR_A 14
#define IN2_MOTOR_A 12

#define ENABLE_MOTOR_B 4
#define IN1_MOTOR_B 15
#define IN2_MOTOR_B 2

#define FRONT_ECHO_PIN 26
#define FRONT_TRIGGER_PIN 27
#define LEFT_ECHO_PIN 18
#define LEFT_TRIGGER_PIN 5
#define RIGHT_ECHO_PIN 25
#define RIGHT_TRIGGER_PIN 33

/************************************************
 *               Motor configuration
 ***********************************************/

motor motorA(ENABLE_MOTOR_A, IN1_MOTOR_A, IN2_MOTOR_A);
motor motorB(ENABLE_MOTOR_B, IN1_MOTOR_B, IN2_MOTOR_B);

/************************************************
 *         Ultrasonic sensor configuration
 ***********************************************/

ultrasonic U_sensorLeft(0, LEFT_ECHO_PIN, LEFT_TRIGGER_PIN); //distance, echo, trigger
ultrasonic U_sensorFront(0, FRONT_ECHO_PIN, FRONT_TRIGGER_PIN);
ultrasonic U_sensorRight(0, RIGHT_ECHO_PIN, RIGHT_TRIGGER_PIN);

/************************************************
 *              ESP32 Configuration
 ***********************************************/
void setup() {
  // Set pin modes for motor A
  pinMode(motorA.in1, OUTPUT);
  pinMode(motorA.in2, OUTPUT);
  pinMode(motorA.en, OUTPUT);

  // Set pin modes for motor B
  pinMode(motorB.in1, OUTPUT);
  pinMode(motorB.in2, OUTPUT);
  pinMode(motorB.en, OUTPUT);

  // Set initial states of the pins (stop the robot)
  digitalWrite(motorA.in1, LOW);
  digitalWrite(motorA.in2, LOW);

  digitalWrite(motorB.in1, LOW);
  digitalWrite(motorB.in2, LOW);

  Serial.begin(BAUD_RATE);
}

/************************************************
 *                  Main loop 
 ***********************************************/
void loop() { //Loop de ejecución
  moveForward(255,motorA, motorB); //Se mueve hacia delante
  float DistanceFront = readUltrasonic(U_sensorFront); //El sensor ultrasónico del frente mide la distancia
  Serial.println(DistanceFront); //Se imprime la distancia para verificar funcionamiento
  if (DistanceFront<=10){ //Si la distancia con una pared enfrente es menor a 10
    float DistanceRight = readUltrasonic(U_sensorRight); //Se mide la distancia con la pared de la derecha
    if (DistanceRight>15){ //Si la distancia con la pared de la derecha es mayor a 15
      //Se inicia el proceso para girar hacia la derecha
      moveBackward(255,motorA,motorB); //Se va hacia atras poquito
      delay(200); //Delay de 200 microsegundos
      rotate90('R',motorA,motorB); //Gira hacia la derecha con la función del template de rotate 90
      moveForward(255,motorA,motorB); //Va hacia el frente
      delay(300); //Por 300 microsegundos 
      rotate90('R',motorA,motorB);//Gira hacia la derecha con la función del template de rotate 90
    }else {
      moveBackward(255,motorA,motorB);//Se va hacia atras poquito
      delay(200);//Delay de 200 microsegundos
      rotate90('L',motorA,motorB);//Gira hacia la izquierda con la función del template de rotate 90
      moveForward(255,motorA,motorB);//Va hacia delante
      delay(300);//Delay de 300 microsegundos
      rotate90('L',motorA,motorB);//Gira hacia la izquierda con la función del template de rotate 90
      }//float DistanceLeft = readUltrasonic(U_sensorLeft);2
    
  } 
  delay(100);//Delay de 100 microsegundos estandar para que el sensor ultrasónico funcione bien
}