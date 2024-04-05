/*
 * Contains all code necessary to control and setup head servos
 */
#include "head.hpp"

//Create objects
//no num is front, 2 is left, 3 is right
Servo frontPlatformServo; //front
Servo leftPlatformServo; //left
Servo rightPlatformServo; //right
Servo yawPlatformServo; //yaw servo
Servo leftEarYawServo; //left ear yaw
Servo leftEarPitchServo; //left ear pitch
Servo rightEarYawServo; //right ear yaw
Servo rightEarPitchServo; //right ear pitch

int angles[3]; //0-front, 1-left, 2-right
int servoAdjust[8]; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
int prevAngles[8];
double currentYaw = 0;
double prevPlatformPos[4]; //0-z, 1-roll, 2-pitch, 4-yaw
double servoDelay;
int iterations;
double unitCubicCoeff[2];

void setupHead(){
  //frontPlatformServo.attach(2); //front
  leftPlatformServo.attach(5); //left
  rightPlatformServo.attach(7); //right
  yawPlatformServo.attach(6); //
  //leftEarYawServo.attach(6); //
  //leftEarPitchServo.attach(7); //
  //rightEarYawServo.attach(8); //
  //rightEarPitchServo.attach(9); //

  //servoAdjust[0] = -5; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  servoAdjust[1] = -9; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  servoAdjust[2] = 7; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  servoAdjust[3] = -4; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  //servoAdjust[4] = 3; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  //servoAdjust[5] = -3; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  //servoAdjust[6] = 0; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  //servoAdjust[7] = -5; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch

  //frontPlatformServo.write(90 + servoAdjust[0]); //front
  leftPlatformServo.write(90 + servoAdjust[1]); //left
  rightPlatformServo.write(90 + servoAdjust[2]); //right
  yawPlatformServo.write(90 + servoAdjust[3]); 
  //leftEarYawServo.write(90 + servoAdjust[4]); 
  //leftEarPitchServo.write(90 + servoAdjust[5]); 
  //rightEarYawServo.write(90 + servoAdjust[6]); 
  //rightEarPitchServo.write(90 + servoAdjust[7]); 
  
  //delay(1000);

  prevAngles[0] = prevAngles[1] = prevAngles[2] = prevAngles[3] = prevAngles[4] = prevAngles[5] = prevAngles[6] = prevAngles[7] = 90;
  //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  
  prevPlatformPos[0] = prevPlatformPos[1] = prevPlatformPos[2] = prevPlatformPos[3] = 0;
  
  //delay(1000);
}

void runHead(double z, double roll, double pitch, double yaw, int LEY, int LEP, int REY, int REP, double iters){
  double ratio;
  double angleDiff[8];
  double positions[8];
  double unitCubicValue;
  positions[0] = z;
  positions[1] = roll;
  positions[2] = pitch;
  positions[3] = yaw;
  positions[4] = LEY;
  positions[5] = LEP;
  positions[6] = REY;
  positions[7] = REP;
  
  unitCubicCreate(iters);

  for(int i = 0; i <= 3; i++){
    angleDiff[i] = positions[i] - prevPlatformPos[i];
  }
  
  for(int i = 4; i <= 7; i++){
    angleDiff[i] = positions[i] - prevAngles[i];
  }

  if(abs(yaw - currentYaw) > 0){ //enter into yaw based control
    
    for(int i = 1; i <= iters; i++){
      ratio = (double)i / iters;

      unitCubicValue = unitCubicCoeff[0] * pow(i, 3) + unitCubicCoeff[1] * pow(i, 2);

      for(int i = 0; i <= 2; i++){ //positions of platform
        positions[i] = angleDiff[i] * unitCubicValue + prevPlatformPos[i];
      }
      
      currentYaw = unitCubicValue * angleDiff[3] + prevPlatformPos[3];
      
      for(int i = 4; i <= 7; i++){ //positions of ear servos
        positions[i] = angleDiff[i] * unitCubicValue + prevAngles[i];
      }
      
      angleCalculate(angles, positions[0], positions[1], positions[2], currentYaw); //z, roll, pitch, yaw

      frontPlatformServo.write(angles[0] + servoAdjust[0]); 
      leftPlatformServo.write(angles[1] + servoAdjust[1]); 
      rightPlatformServo.write(angles[2] + servoAdjust[2]); 
      yawPlatformServo.write(currentYaw + 90 + servoAdjust[3]); 
      leftEarYawServo.write(positions[4] + servoAdjust[4]); 
      leftEarPitchServo.write(positions[5] + servoAdjust[5]); 
      rightEarYawServo.write(positions[6] + servoAdjust[6]); 
      rightEarPitchServo.write(positions[7] + servoAdjust[7]);
//      delay(servoDelay); 
    }

    for(int i = 0; i <= 2; i++){ //when done, make previous angles the end position of the motion
      prevAngles[i] = angles[i];
    }
    
    for(int i = 3; i <= 7; i++){ //when done, make previous angles the end position of the motion
      prevAngles[i] = positions[i];
    }
  }
  
  else{ //enter into no yaw basted control

  angleCalculate(angles, positions[0], positions[1], positions[2], positions[3]); //z, roll, pitch, yaw
  
    for(int i = 1; i <= iters; i++){
      ratio = (double)i / iters;

      unitCubicValue = unitCubicCoeff[0] * pow(i, 3) + unitCubicCoeff[1] * pow(i, 2);

      for(int i = 0; i <= 2; i++){ //positions of platform servos
        positions[i] = angles[i] * unitCubicValue + prevAngles[i] * (1 - unitCubicValue);
      }
      for(int i = 4; i <= 7; i++){ //positions of ear and yaw servos
        positions[i] = angleDiff[i] * unitCubicValue + prevAngles[i];
      }

      frontPlatformServo.write(positions[0] + servoAdjust[0]); 
      leftPlatformServo.write(positions[1] + servoAdjust[1]); 
      rightPlatformServo.write(positions[2] + servoAdjust[2]); 
      //yawPlatformServo.write(positions[3] + servoAdjust[3]); 
      leftEarYawServo.write(positions[4] + servoAdjust[4]); 
      leftEarPitchServo.write(positions[5] + servoAdjust[5]); 
      rightEarYawServo.write(positions[6] + servoAdjust[6]); 
      rightEarPitchServo.write(positions[7] + servoAdjust[7]);
//      delay(servoDelay); 
    }

    for(int i = 0; i <= 7; i++){ //when done, make previous angles the end position of the motion
      prevAngles[i] = positions[i];
    }
  }

  prevPlatformPos[0] = z;
  prevPlatformPos[1] = roll;
  prevPlatformPos[2] = pitch;
  prevPlatformPos[3] = yaw;
}

void unitCubicCreate(double tf){
  double a, b; //a is cubic coeff, b is squared coeff

  a = -2 / pow(tf, 3);
  b = 3 / pow(tf, 2);

  unitCubicCoeff[0] = a;
  unitCubicCoeff[1] = b;
  
}

void angleCalculate(int arr[3], double z, double roll, double pitch, double yaw){
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
  // This sets the servo angle to the outgoing array.
  arr[0] = servoAngle1 * radToDeg;

  // This takes in the x and y positions for the left side
  y = 25*sin(pitch) + 115 + ((30*sin(-roll))*cos(pitch));
  x = 15 - (30 - (30*cos(-roll)));
  // This calculates the joint and servo angles for the left side.
  jointAngle2 = acos((pow(x,2) + pow(y,2) - pow(17,2) - pow(104,2))/(2 * 17 * 104));
  servoAngle2 = atan(y/x) - atan((104*sin(jointAngle2))/(17+104*cos(jointAngle2)));
  // This defines the servo angle in the outgoing array.
  arr[1] = servoAngle2 * radToDeg;
  // The yaw is then inserted into the third position of the outgoing array
  arr[2] = yaw;
}

void testHeadLoop(){
  iterations = 100;
  servoDelay = 2;
  //runPlatform(0, 0, 0, 0, 0); //z, roll, pitch, yaw, no yaw iterations
  
  /*runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 30, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 30, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 60, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90-60, 90+60, 90+60, 90-60, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations*/

  runHead(0, 0, 30, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 30, 90, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 30, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations

  /*runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 35, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, -35, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, -35, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 35, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, -25, -25, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 25, 25, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations*/
  
  /*leftEarYawServo.write(180); 
  leftEarPitchServo.write(180); 
  rightEarYawServo.write(180); 
  rightEarPitchServo.write(180); 
  delay(500);
  yawPlatformServo.write(90);
  leftEarYawServo.write(90); 
  leftEarPitchServo.write(90); 
  rightEarYawServo.write(90); 
  rightEarPitchServo.write(90); 
  delay(500);
  yawPlatformServo.write(0);
  leftEarYawServo.write(0); 
  leftEarPitchServo.write(0); 
  rightEarYawServo.write(0); 
  rightEarPitchServo.write(0); 
  delay(500);
  yawPlatformServo.write(90);
  leftEarYawServo.write(90); 
  leftEarPitchServo.write(90); 
  rightEarYawServo.write(90); 
  rightEarPitchServo.write(90); 
  delay(500);*/

  /*frontPlatformServo.write(90 + servoAdjust[0]); 
  leftPlatformServo.write(90 + servoAdjust[1]); 
  rightPlatformServo.write(90 + servoAdjust[2]); 
  yawPlatformServo.write(90 + servoAdjust[3]); 
  leftEarYawServo.write(90 + servoAdjust[4]); 
  leftEarPitchServo.write(90 + servoAdjust[5]); 
  rightEarYawServo.write(90 + servoAdjust[6]); 
  rightEarPitchServo.write(90 + servoAdjust[7]);
  delay(500);
  frontPlatformServo.write(110 + servoAdjust[0]); 
  leftPlatformServo.write(110 + servoAdjust[1]); 
  rightPlatformServo.write(110 + servoAdjust[2]); 
  yawPlatformServo.write(110 + servoAdjust[3]); 
  leftEarYawServo.write(110 + servoAdjust[4]); 
  leftEarPitchServo.write(110 + servoAdjust[5]); 
  rightEarYawServo.write(110 + servoAdjust[6]); 
  rightEarPitchServo.write(110 + servoAdjust[7]);
  delay(500);*/
}
