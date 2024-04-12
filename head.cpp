/*
 * Contains all code necessary to control and setup head servos
 */
#include "head.hpp"

//Create objects
//no num is front, 2 is left, 3 is right

//Servo frontPlatformServo; //front

Servo leftPlatformServo; //left
Servo rightPlatformServo; //right
Servo yawPlatformServo; //yaw servo

/*Servo leftEarYawServo; //left ear yaw
Servo leftEarPitchServo; //left ear pitch
Servo rightEarYawServo; //right ear yaw
Servo rightEarPitchServo; //right ear pitch
*/

int angles[3]; //0-front, 1-left, 2-right

int servoAdjust[3]; //0-Left, 1-Right, 2-Yaw

int prevAngles[3];

double currentYaw = 0;

double prevPlatformPos[3]; // 0-roll, 1-pitch, 2-yaw

double servoDelay;

int iterations;


void setupHead(){
 
  leftPlatformServo.attach(5); // left
  rightPlatformServo.attach(7); // right
  yawPlatformServo.attach(6); // yaw


  leftPlatformServo.write(LEFT_POS_DEFAULT);    // left
  rightPlatformServo.write(RIGHT_POS_DEFAULT);  // right
  yawPlatformServo.write(YAW_POS_DEFAULT);      // yaw

  

  prevAngles[0] = prevAngles[1] = prevAngles[2] = 90;
  //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  
  prevPlatformPos[0] = prevPlatformPos[1] = prevPlatformPos[2] = 0;
  
  //delay(1000);
}

void runHead(double roll, double pitch, double yaw, double iters){
  double ratio;
  double angleDiff[3];
  double positions[3];

  positions[0] = roll;
  positions[1] = pitch;
  positions[2] = yaw;

  angleCalculate(angles, roll, pitch, yaw);

  leftPlatformServo.write(angles[0]);
  rightPlatformServo.write(angles[1]);
  yawPlatformServo.write(angles[2]);


  prevPlatformPos[0] = roll;
  prevPlatformPos[1] = pitch;
  prevPlatformPos[2] = yaw;

  prevAngles[0] = angles[0];
  prevAngles[1] = angles[1];
  prevAngles[2] = angles[2];

}

void angleCalculate(int arr[3], double roll, double pitch, double yaw){
  // These define the joint and servo angles for the right side
  double jointAngle1,servoAngle1;
  
  // These define the joint and servo angles for the left side
  double jointAngle2,servoAngle2;
  
  // These are used to convert between degrees and radians.
  double radToDeg = 180 / M_PI;
  double degToRad = M_PI / 180;
  
  // This converts the pitch and roll to radians
  pitch = pitch * degToRad;
  roll = roll * degToRad;
  
  // This defines the x and y positions
  double x,y;
  
  // This calculates the x and y positions of the right side
  y = 25*sin(pitch) + 115 + ((30*sin(roll))*cos(pitch));
  x = 15 - (30 - (30*cos(roll)));
  
  // This calculates the joint and servo angle for the right side
  jointAngle1 = acos((pow(x,2) + pow(y,2) - pow(17,2) - pow(104,2))/(2 * 17 * 104));
  servoAngle1 = atan(y/x) - atan((104*sin(jointAngle1))/(17+104*cos(jointAngle1)));
  
  // This sets the R_HEAD servo angle to the outgoing array.
  arr[0] = RIGHT_POS_DEFAULT - ((servoAngle1 * radToDeg) - 40);

  // This takes in the x and y positions for the left side
  y = 25*sin(pitch) + 115 + ((30*sin(-roll))*cos(pitch));
  x = 15 - (30 - (30*cos(-roll)));
  
  // This calculates the joint and servo angles for the left side.
  jointAngle2 = acos((pow(x,2) + pow(y,2) - pow(17,2) - pow(104,2))/(2 * 17 * 104));
  servoAngle2 = atan(y/x) - atan((104*sin(jointAngle2))/(17+104*cos(jointAngle2)));
  
  // This defines the L_HEAD servo angle in the outgoing array.
  arr[1] = LEFT_POS_DEFAULT + ((servoAngle2 * radToDeg) - 40);
  
  // The yaw is then inserted into the third position of the outgoing array
  arr[2] = yaw + YAW_POS_DEFAULT;
}

void testHeadLoop(){
  iterations = 100;
  servoDelay = 2;
  
  runHead(0, 30, 0, iterations); //roll, pitch, yaw, iterations
  runHead(0, 30, 90, iterations); //roll, pitch, yaw, iterations
  runHead(0, 30, 0, iterations); //roll, pitch, yaw, iterations
  runHead(0, 0, 0, iterations); //roll, pitch, yaw, iterations

  
}
