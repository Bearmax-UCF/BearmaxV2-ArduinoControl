#include <Arduino.h>
#include <Servo.h> // servo library

#define CHASSIS_PIN 30
#define RIGHT_ARM_SHOULDER_PIN 8
#define RIGHT_ARM_ROTATOR_PIN 9
#define RIGHT_ARM_ELBOW_PIN 10
#define LEFT_ARM_SHOULDER_PIN 3
#define LEFT_ARM_ROTATOR_PIN 4
#define LEFT_ARM_ELBOW_PIN 2
#define RIGHT_HAND_PAW_PIN 22
#define LEFT_HAND_PAW_PIN 13
#define RIGHT_HAND_GRIP_PIN 20
#define LEFT_HAND_GRIP_PIN 11
#define RIGHT_HAND_FINGER_PIN 21
#define LEFT_HAND_FINGER_PIN 12

#ifndef BAUD_RATE
#define BAUD_RATE 57600
#endif

void setupBody();

void runBody(int chassis, int las, int lar, int lae, int ras, int rar, int rae);
