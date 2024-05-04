/**
 * @file sensor.h
 * @brief Ultrasonic Sensor Control Library
 * @author David Ortiz
 * @date 04/08/2024
 * 
 * @brief Description:
 * This file contains useful functions for controlling the ultrasonic sensor 
 * of the robot.
 */

#ifndef SENSOR_READINGS_H
#define SENSOR_READINGS_H

#include <Arduino.h>

//define sound speed in cm/uS
#define SOUND_SPEED 0.034

/**
 * @struct motor
 * @brief Brief description of the struct.
 *
 * Detailed description of the struct.
 */
struct ultrasonic{
    long distance;
    const int echo;
    const int trigger;

    ultrasonic(long distanceVal, int echoVal, int triggerVal)
        : distance(distanceVal), echo(echoVal), trigger(triggerVal){
    }
};

// Turn the sensor on, get a reading, convert time into distance
void readUltrasonic(ultrasonic &U_sensor);

#endif // SENSOR_READINGS_H