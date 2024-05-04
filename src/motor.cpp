/**
 * @file motor.cpp
 * @brief Motor Control Library - Implementation
 * @author Pablo Vargas & David Ortiz
 * @date 04/08/2024
 * 
 * @brief Description:
 * This file contains useful functions for controlling the motors of the robot. 
 * It utilizes a struct for each motor and provides functions for simple robot 
 * translation and rotation.
 * 
 * @brief Credits:
 * Some of the code functions in this library were adapted from the following sources:
 * - https://github.com/ParzivalStark/TallerIRS by ParzivalStark
 * - https://www.instructables.com/Arduino-Flashlight-Following-Robot/ by joesinstructables
 * Proper credit goes to them for their contributions.
 */

#include <Arduino.h>
#include "../include/motor.h"


#define ROTATE90_TIME  390
#define ROTATE90_PWM_INTENSITY  200
#define ROTATE90ONE_TIME 1000
#define ROTATE90ONE_PWM_INTENSITY 200

#define ROTATE_LEAD_MOTOR 165
#define ROTATE_FOLLOW_MOTOR 130

/************************************************
 *  Common movement functions and rotations
 ***********************************************/

void moveForward(int speed, motor &motorA, motor &motorB){
  analogWrite(motorA.en, 180);
  analogWrite(motorB.en, speed);

  digitalWrite(motorA.in1, HIGH);
  digitalWrite(motorA.in2, LOW);

  digitalWrite(motorB.in1, HIGH);
  digitalWrite(motorB.in2, LOW);
}


void rotateLeft_axis(int speed, motor &motorA, motor &motorB) {
    analogWrite(motorA.en, speed);
    analogWrite(motorB.en, speed);

    digitalWrite(motorA.in1, HIGH);
    digitalWrite(motorA.in2, LOW);

    digitalWrite(motorB.in1, LOW);
    digitalWrite(motorB.in2, HIGH);
}


void rotateRight_axis(int speed, motor &motorA, motor &motorB) {
    analogWrite(motorA.en, speed);
    analogWrite(motorB.en, speed);

    digitalWrite(motorA.in1, LOW);
    digitalWrite(motorA.in2, HIGH);

    digitalWrite(motorB.in1, HIGH);
    digitalWrite(motorB.in2, LOW);
}


void rotateRight_one(int speed, motor &motorA, motor &motorB) {
    analogWrite(motorA.en, speed);
    analogWrite(motorB.en, speed);

    digitalWrite(motorA.in1, LOW);
    digitalWrite(motorA.in2, LOW);

    digitalWrite(motorB.in1, HIGH);
    digitalWrite(motorB.in2, LOW);
}


void rotateLeft_one(int speed, motor &motorA, motor &motorB) {
    analogWrite(motorA.en, speed);
    analogWrite(motorB.en, speed);

    digitalWrite(motorA.in1, HIGH);
    digitalWrite(motorA.in2, LOW);

    digitalWrite(motorB.in1, LOW);
    digitalWrite(motorB.in2, LOW);
}


void moveBackward(int speed, motor &motorA, motor &motorB) {
    analogWrite(motorA.en, 190);
    analogWrite(motorB.en, speed);

    digitalWrite(motorA.in1, LOW);
    digitalWrite(motorA.in2, HIGH);

    digitalWrite(motorB.in1, LOW);
    digitalWrite(motorB.in2, HIGH);
}


void stop(motor &motorA, motor &motorB) {
    digitalWrite(motorA.in1, LOW);
    digitalWrite(motorA.in2, LOW);

    digitalWrite(motorB.in1, LOW);
    digitalWrite(motorB.in2, LOW);
}


void rotateLeft_radius(motor &motorA, motor &motorB){
    analogWrite(motorA.en, ROTATE_LEAD_MOTOR);
    analogWrite(motorB.en, ROTATE_FOLLOW_MOTOR);

    digitalWrite(motorA.in1, HIGH);
    digitalWrite(motorA.in2, LOW);

    digitalWrite(motorB.in1, HIGH);
    digitalWrite(motorB.in2, LOW);
}


void rotateRight_radius(motor &motorA, motor &motorB){
    analogWrite(motorA.en, ROTATE_FOLLOW_MOTOR);
    analogWrite(motorB.en, ROTATE_LEAD_MOTOR);

    digitalWrite(motorA.in1, HIGH);
    digitalWrite(motorA.in2, LOW);

    digitalWrite(motorB.in1, HIGH);
    digitalWrite(motorB.in2, LOW);
}


void rotate90(char direction, motor &motorA, motor &motorB){
    // Adjust speed and time for the robots to rotate 90 degrees
    if(direction == 'L' || direction == 'l'){
        rotateLeft_axis(ROTATE90_PWM_INTENSITY, motorA, motorB);
        delay(450);
        stop(motorA, motorB);
    }else if(direction == 'R' || direction == 'r'){
        rotateRight_axis(ROTATE90_PWM_INTENSITY, motorA, motorB);
        delay(ROTATE90_TIME);
        stop(motorA, motorB);
    }else{
        Serial.print("Not a valid rotation direction.");
    }   
}


void rotate90_one(char direction, motor &motorA, motor &motorB){
    // Adjust speed and time for the robots to rotate 90 degrees
    if(direction == 'L' || direction == 'l'){
        rotateRight_one(ROTATE90ONE_PWM_INTENSITY, motorA, motorB);
        delay(ROTATE90ONE_TIME);
        stop(motorA, motorB);
    }else if(direction == 'R' || direction == 'r'){
        rotateLeft_one(ROTATE90ONE_PWM_INTENSITY, motorA, motorB);
        delay(ROTATE90ONE_TIME);
        stop(motorA, motorB);
    }else{
        Serial.print("Not a valid rotation direction.");
    }   
}

void rotate_radius(char direction, motor &motorA, motor &motorB){
    // Adjust speed and time for the robots to rotate 90 degrees
    if(direction == 'L' || direction == 'l'){
        rotateRight_radius(motorA, motorB);
        delay(ROTATE90ONE_TIME);
        stop(motorA, motorB);
    }else if(direction == 'R' || direction == 'r'){
        rotateLeft_radius(motorA, motorB);
        delay(ROTATE90ONE_TIME);
        stop(motorA, motorB);
    }else{
        Serial.print("Not a valid rotation direction.");
    }   
}