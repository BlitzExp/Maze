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

#define ENABLE_MOTOR_B 14
#define IN1_MOTOR_B 15
#define IN2_MOTOR_B 12

#define LEFT_ECHO_PIN 26
#define LEFT_TRIGGER_PIN 27
#define FRONT_ECHO_PIN 18
#define FRONT_TRIGGER_PIN 5
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
void loop() {

}