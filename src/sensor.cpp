/**
 * @file sensor.cpp
 * @brief Ultrasonic Sensor Control Library - Implementation
 * @author David Ortiz
 * @date 04/08/2024
 * 
 * @brief Description:
 * This file contains useful functions for controlling the ultrasonic sensor 
 * of the robot.
 */

#include <Arduino.h>
#include "../include/sensor.h"


void readUltrasonic(ultrasonic &U_sensor){
    pinMode(U_sensor.trigger, OUTPUT); // Sets the trigPin as an Output
  	pinMode(U_sensor.echo, INPUT); // Sets the echoPin as an Input
	digitalWrite(U_sensor.trigger, LOW); // Clears the trigPin
	delayMicroseconds(2);

	digitalWrite(U_sensor.trigger, HIGH);
	delayMicroseconds(10);
	digitalWrite(U_sensor.trigger, LOW);

	long duration = pulseIn(U_sensor.echo, HIGH);

	U_sensor.distance = duration * SOUND_SPEED/2;
}
