/*
 * Contains the code necessary to communicate with ROS master.
 */
#include "head.hpp"
#include "body.hpp"

// This is to make switching between the production Due and other Arduinos used for development easier
#ifdef SerialUSB
  #define SerialConn SerialUSB
#else
  #define SerialConn Serial
#endif

#define BAUD_RATE 57600

// Used for array indexes! Don't change numbers!
enum Joint {
  CHASSIS = 0,
  L_ARM_SHOULDER,
  L_ARM_ROTATOR,
  L_ARM_ELBOW,
  L_ARM_GRIP,
  L_ARM_THUMB,
  L_ARM_PAW,
  R_ARM_SHOULDER,
  R_ARM_ROTATOR,
  R_ARM_ELBOW,
  R_ARM_GRIP,
  R_ARM_THUMB,
  R_ARM_PAW,
  L_HEAD,
  R_HEAD,
  HEAD_YAW,
  NUMBER_OF_JOINTS /* not a valid index! */
};

/*
 * Joint states, as int. In degrees from 0 - 180
 * Indexed by Joint enum
 */
int prev_target_state[Joint::NUMBER_OF_JOINTS];
int target_state[Joint::NUMBER_OF_JOINTS];

void setup() {
  SerialConn.begin(BAUD_RATE, SERIAL_8N1);
  Serial.begin(BAUD_RATE, SERIAL_8N1);
  // Old version
  //Serial1.begin(BAUD_RATE, SERIAL_8N1);
  while (!SerialConn) {
    ; // Wait for serial port to connect.
  }

  SerialConn.println("Start H Setup");
  // Run setup for Head
  setupHead();
  SerialConn.println("End H Setup");

  SerialConn.println("Start Body Setup");
  // TODO: Run setup for arms & body
  setupBody();
  SerialConn.println("End Body Setup");
  
  // Send ready packet to master
  SerialConn.println("ready");
   
  // 0-Chassis, 1-L_Shoulder, 2-L_Rotator, 3-L_Elbow, 4-L_Grip, 5-L_Thumb, 6-L_Paw, 7-R_Shoulder, 8-R_Rotator, 9-R_Elbow, 10-R_Grip, 11-R_Thumb, 12-R_Paw
  // Delays Below
  //      0-0,  1-180,  2-90, 3-0,  4-90, 5-0,  6-90, 7-0,  8-90, 9-180,  10-0, 11-90, 12-0
  runBody(  0,      0,     0,   0,     0,   0,     0,   0,     0,     0,     0,     0,    0);

  // Which way each joint moves:
  /** s 0:180:90:0:90:0:90:0:90:180:0:90:0:0:0:0
   * 0- 0-180, Increase
   * LEFT ARM BELOW
   * 1- 0-180, Increase angle = lower position,       180 neutral pos
   * 2- 0-180, Increase angle = thumb points inward,   90 neutral pos
   * 3- 0-180, Increase angle = higher elbow position,  0 neutral pos
   * 4- 0-180, Increase angle = grip closes,           90 neutral pos
   * 5- 0-180, Increase angle = thumb outward,          0 neutral pos
   * 6- 0-180, Increase angle = paw closes,            90 neutral pos
   * RIGHT ARM BELOW
   * 7- 0-180, Increase angle = higher position,        0 neutral pos
   * 8- 0-180, Increase angle = thumb points outward,  90 neutral pos
   * 9- 0-180, Increase angle = elbow points down,    180 neutral pos
   * 10-0-180, Increase angle = grip opens more,        0 neutral pos 
   * 11-0-180, Increase angle = thumb points in more,  90 neutral pos
   * 12-0-180, Increase angle = palm opens up,          0 neutral pos
   * 
   */
  


  
}

/**
 * Converts joint state array into string formatted as 0:0:0:...
 */
String serialize_joint_states(int *s) {
  String tmp = "";

  for (int i = 0; i < Joint::NUMBER_OF_JOINTS; i++) {
    tmp += s[i];
    if (i + 1 != Joint::NUMBER_OF_JOINTS) {
      tmp += ":"; // Don't add ':' to final joint value
    }
  }
  
  return tmp;
}

/**
 * Converts joint state string to joint state array
 * @param str trimmed string formatted as 0:0:0:0:...
 * @param *s int array to save joint states into
 */
void deserialize_joint_states(String str, int s[]) {
  int delim_idx = -1; // Index of the first delimeter is the start of the string
  
  for (int i = 0; i < Joint::NUMBER_OF_JOINTS; i++) {
    
    int next_delim_idx = str.indexOf(':', delim_idx + 1);

    s[i] = (int) str.substring(delim_idx + 1, next_delim_idx).toInt();
    
    delim_idx = next_delim_idx;
  }
}

// Copies target_state into prev_target_state
void save_prev_target_state() {
  for (int i = 0; i < Joint::NUMBER_OF_JOINTS; i++) {
    prev_target_state[i] = target_state[i];
  }
}

// Handles commands sent from serial port.
void cmd_handler() {
  //SerialConn.println("Dont be Here");
  if (SerialConn.available() > 0) {
    SerialConn.println("Here");
    String cmd = SerialConn.readStringUntil('\r');
    cmd.trim();
    if (cmd.startsWith("s ")) {
      String raw_values = cmd.substring(2);
      raw_values.trim();
      save_prev_target_state();
      deserialize_joint_states(raw_values, target_state);
      handle_new_target();
      Serial.println(cmd);
      // Acknowledge command
      SerialConn.print("ack: ");
      SerialConn.println(cmd);
    } else if (cmd.startsWith("r")) {
      // WIP: If ros doesn't smooth positions well and we need to use velocity as well,
      //      then we need to read current_state, instead of target.
      String state_string = serialize_joint_states(target_state);
      SerialConn.println(state_string);
    } else {
      //SerialConn.print("[Error]: Invalid Command: ");
      //SerialConn.println(cmd);
    }
  }
  //SerialConn.println("ESP Dont be Here");
}

void handle_new_target() {

  // If one of the head joints has been updated, execute.
  if (head_has_update()) {
    //SerialConn.println("hi");
    // iterations are 0 bkz we're assuming that ROS is sending real-time positions that will do its own smoothing.
    runHead(
      (double) target_state[Joint::L_HEAD],
      (double) target_state[Joint::R_HEAD],
      (double) target_state[Joint::HEAD_YAW],
      1 /* 1 iterations */
    );
  }
  
  if (body_has_update()) {
    runBody(
      target_state[Joint::CHASSIS],
      target_state[Joint::L_ARM_SHOULDER],
      target_state[Joint::L_ARM_ROTATOR],
      target_state[Joint::L_ARM_ELBOW],
      target_state[Joint::L_ARM_GRIP],
      target_state[Joint::L_ARM_THUMB],
      target_state[Joint::L_ARM_PAW],
      target_state[Joint::R_ARM_SHOULDER],
      target_state[Joint::R_ARM_ROTATOR],
      target_state[Joint::R_ARM_ELBOW],
      target_state[Joint::R_ARM_GRIP],
      target_state[Joint::R_ARM_THUMB],
      target_state[Joint::R_ARM_PAW]
     );
  }
}

#define NUM_OF_HEAD_JOINTS 3
int head_joint_idxs[NUM_OF_HEAD_JOINTS] = {
  Joint::L_HEAD,
  Joint::R_HEAD,
  Joint::HEAD_YAW
};

bool head_has_update() {
  for (int i = 0; i < NUM_OF_HEAD_JOINTS; i++) {
    int joint_idx = head_joint_idxs[i];
    if (prev_target_state[joint_idx] != target_state[joint_idx]) {
      return true;
    }
  }
  return false;
}

#define NUM_OF_BODY_JOINTS 13
int body_joint_idxs[NUM_OF_BODY_JOINTS] = {
  Joint::CHASSIS,
  Joint::L_ARM_SHOULDER,
  Joint::L_ARM_ROTATOR,
  Joint::L_ARM_ELBOW,
  Joint::L_ARM_GRIP,
  Joint::L_ARM_THUMB,
  Joint::L_ARM_PAW,
  Joint::R_ARM_SHOULDER,
  Joint::R_ARM_ROTATOR,
  Joint::R_ARM_ELBOW,
  Joint::R_ARM_GRIP,
  Joint::R_ARM_THUMB,
  Joint::R_ARM_PAW
};

bool body_has_update() {
  for (int i = 0; i < NUM_OF_BODY_JOINTS; i++) {
    int joint_idx = body_joint_idxs[i];
    if (prev_target_state[joint_idx] != target_state[joint_idx]) {
      return true;
    }
  }
  return false;
}

void loop() {
  cmd_handler();   
}

  // Pose angle guide for All poses (Simple versions):
  // These angles are what ROS2 sends over converted to degrees, subject to change
  /**
   * HAPPY
   * Chassis:       0
   * L_Shoulder:    140
   * L_Rotator:     0
   * L_Elbow:       70
   * L_Grip:        86
   * L_Thumb:       0
   * L_Paw:         0
   * R_Shoulder:    120
   * R_Rotator:     0
   * R_Elbow:       82
   * R_Grip:        0
   * R_Thumb:       0
   * R_Paw:         0
   * L_Neck:        0
   * R_Neck:        0
   * Head_Yaw:      0
   * 
   * SAD                s 0:105:30:70:90:0:0:120:60:50:90:0:0:0:10:0
   * Chassis:       0
   * L_Shoulder:    105
   * L_Rotator:     30
   * L_Elbow:       70
   * L_Grip:        90
   * L_Thumb:       0
   * L_Paw:         0
   * R_Shoulder:    120
   * R_Rotator:     60
   * R_Elbow:       50
   * R_Grip:        90
   * R_Thumb:       0
   * R_Paw:         0
   * L_Neck:        0
   * R_Neck:        10
   * Head_Yaw:      0
   * 
   * ANGRY            s 22:90:45:64:90:0:80:90:45:64:90:0:90:0:0:0
   * Chassis:       22
   * L_Shoulder:    90
   * L_Rotator:     45
   * L_Elbow:       64
   * L_Grip:        90
   * L_Thumb:       0
   * L_Paw:         80
   * R_Shoulder:    90
   * R_Rotator:     45
   * R_Elbow:       64
   * R_Grip:        90
   * R_Thumb:       0
   * R_Paw:         90
   * L_Neck:        0
   * R_Neck:        0
   * Head_Yaw:      0
   * 
   * CONFUSED            s 0:120:0:50:44:0:28:0:0:0:0:0:0:10:0:0
   * Chassis:       0
   * L_Shoulder:    155
   * L_Rotator:     0
   * L_Elbow:       0
   * L_Grip:        44
   * L_Thumb:       0
   * L_Paw:         28
   * R_Shoulder:    0
   * R_Rotator:     0
   * R_Elbow:       0
   * R_Grip:        0
   * R_Thumb:       0
   * R_Paw:         0
   * L_Neck:        10
   * R_Neck:        0
   * Head_Yaw:      0
   * 
   * SHOCKED            s 0:150:0:20:0:0:0:150:0:20:0:0:0:0:0:0
   * Chassis:       0
   * L_Shoulder:    150
   * L_Rotator:     0
   * L_Elbow:       20
   * L_Grip:        0
   * L_Thumb:       0
   * L_Paw:         0
   * R_Shoulder:    150
   * R_Rotator:     0
   * R_Elbow:       20
   * R_Grip:        0
   * R_Thumb:       0
   * R_Paw:         0
   * L_Neck:        0
   * R_Neck:        0
   * Head_Yaw:      0
   * 
   * //MOVE NECK AS WELL FOR THIS
   * WORRIED            s 0:95:40:50:0:0:0:95:50:53:0:0:0:5:0:0
   * Chassis:       0
   * L_Shoulder:    65
   * L_Rotator:     32
   * L_Elbow:       66
   * L_Grip:        0
   * L_Thumb:       0
   * L_Paw:         0
   * R_Shoulder:    95
   * R_Rotator:     0
   * R_Elbow:       53
   * R_Grip:        0
   * R_Thumb:       0
   * R_Paw:         66
   * L_Neck:        10
   * R_Neck:        0
   * Head_Yaw:      0
   * 
   * SCARED            s 55:85:53:74:90:0:0:85:50:80:0:0:0:0:15:30
   * Chassis:       55
   * L_Shoulder:    85
   * L_Rotator:     53
   * L_Elbow:       74
   * L_Grip:        90
   * L_Thumb:       0
   * L_Paw:         0
   * R_Shoulder:    85
   * R_Rotator:     50
   * R_Elbow:       80
   * R_Grip:        0
   * R_Thumb:       0
   * R_Paw:         0
   * L_Neck:        0
   * R_Neck:        15
   * Head_Yaw:      30
   * 
   * ANNOYED            s 22:60:60:88:0:0:0:36:60:80:0:0:0:0:0:32
   * Chassis:       22
   * L_Shoulder:    66
   * L_Rotator:     60
   * L_Elbow:       88
   * L_Grip:        0
   * L_Thumb:       0
   * L_Paw:         0
   * R_Shoulder:    36
   * R_Rotator:     60
   * R_Elbow:       88
   * R_Grip:        0
   * R_Thumb:       0
   * R_Paw:         0
   * L_Neck:        0
   * R_Neck:        0
   * Head_Yaw:      32
  */
