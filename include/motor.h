/**
 * @file motor.h
 * @brief Motor Control Library
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

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

/**
 * @struct motor
 * @brief Represents a physical motor of the robot.
 * Enables management and control, via the use of a H-bridge.
 *
 * @param en Enable the motor.
 * @param in1 Connected to one of the input pins of the H-bridge.
 * @param in2 Connected the other one of the input pins of the H-bridge.
 */
struct motor {
    int en;
    int in1;
    int in2;

    // Constructor for the motor struct
    motor(int enValue, int in1Value, int in2Value) {
        en = enValue;
        in1 = in1Value;
        in2 = in2Value;
    }
};

/************************************************
 *  Common distance movement and rotations
 ***********************************************/

/**
 * @brief Activates motorA and motorB, with the same intensity and direction. Makes the robot
 * move forward.
 * @param speed Input for analogWrite of the motors.
 * @param motorA Struct of the motor A object.
 * @param motorB Struct of the motor B object.
*/
void moveForward(int speed, motor &motorA, motor &motorB);


/**
 * @brief Activates motorA and motorB, with the different intensity and direction. Makes the robot
 * rotate left, on it's axis.
 * @param speed Input for analogWrite of the motors.
 * @param motorA Struct of the motor A object.
 * @param motorB Struct of the motor B object.
*/
void rotateLeft_axis(int speed, motor &motorA, motor &motorB);


/**
 * @brief Activates motorA and motorB, with the different intensity and direction. Makes the robot
 * rotate right, on it's axis.
 * @param speed Input for analogWrite of the motors.
 * @param motorA Struct of the motor A object.
 * @param motorB Struct of the motor B object.
*/
void rotateRight_axis(int speed, motor &motorA, motor &motorB);


/**
 * @brief Activates motorB. Makes the robot rotate right, pivoting on the right wheel.
 * @param speed Input for analogWrite of the motors.
 * @param motorA Struct of the motor A object.
 * @param motorB Struct of the motor B object.
*/
void rotateRight_one(int speed, motor &motorA, motor &motorB);


/**
 * @brief Activates motorA. Makes the robot rotate left, pivoting on the left wheel.
 * @param speed Input for analogWrite of the motors.
 * @param motorA Struct of the motor A object.
 * @param motorB Struct of the motor B object.
*/
void rotateLeft_one(int speed, motor &motorA, motor &motorB);


/**
 * @brief Activates motorA and motorB, with the same intensity and direction. Makes the robot
 * move backward.
 * @param speed Input for analogWrite of the motors.
 * @param motorA Struct of the motor A object.
 * @param motorB Struct of the motor B object.
*/
void moveBackward(int speed, motor &motorA, motor &motorB);


/**
 * @brief Activates motorA and motorB, with the same intensity and direction. Makes the robot
 * move forward.
 * @param speed Input for analogWrite of the motors.
 * @param motorA Struct of the motor A object.
 * @param motorB Struct of the motor B object.
*/
void rotateLeft_radius(motor &motorA, motor &motorB);


/**
 * @brief Activates motorA and motorB, but with different intensity each. 
 * May be used to avoid potential collisions with nearby walls on the right direction. 
 * @param motorA Struct of the motor A object.
 * @param motorB Struct of the motor B object.
*/
void rotateRight_radius(motor &motorA, motor &motorB);


/**
 * @brief Rotates the robot with both wheels moving in opposite direction, and uses macros for intensity and time of the rotation.
 * @param direction Input for deciding where to rotate: left[l/L] or right[r/R].
 * @param motorA Struct of the motor A object.
 * @param motorB Struct of the motor B object.
*/
void rotate90(char direction, motor &motorA, motor &motorB);


/**
 * @brief Rotates with only one wheel, and uses macros for intensity and time of the rotation.
 * @param direction Input for deciding where to rotate: left[l/L] or right[r/R].
 * @param motorA Struct of the motor A object.
 * @param motorB Struct of the motor B object.
*/
void rotate90_one(char direction, motor &motorA, motor &motorB);

void rotate_radius(char direction, motor &motorA, motor &motorB);

#endif // MOTOR_CONTROL_H