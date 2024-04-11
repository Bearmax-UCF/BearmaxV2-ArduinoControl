
#include "body.hpp"

Servo chassisServo;
Servo lasServo;
Servo larServo;
Servo laeServo;
Servo lagServo;
Servo latServo;
Servo lapServo;

Servo rasServo;
Servo rarServo;
Servo raeServo;
Servo ragServo;
Servo ratServo;
Servo rapServo;

void setupBody() {
    //chassisServo.attach(CHASSIS_PIN);
    
    lasServo.attach(LEFT_ARM_SHOULDER_PIN);
    larServo.attach(LEFT_ARM_ROTATOR_PIN);
    laeServo.attach(LEFT_ARM_ELBOW_PIN);
    
    lagServo.attach(LEFT_HAND_GRIP_PIN);
    latServo.attach(LEFT_HAND_FINGER_PIN);
    lapServo.attach(LEFT_HAND_PAW_PIN);
    


    rasServo.attach(RIGHT_ARM_SHOULDER_PIN);
    rarServo.attach(RIGHT_ARM_ROTATOR_PIN);
    raeServo.attach(RIGHT_ARM_ELBOW_PIN);

    
    ragServo.attach(RIGHT_HAND_GRIP_PIN);
    ratServo.attach(RIGHT_HAND_FINGER_PIN);
    rapServo.attach(RIGHT_HAND_PAW_PIN);
    
    

    //chassisServo.write(90);
    
    /*lasServo.write(90);
    larServo.write(90);
    laeServo.write(90);
    rasServo.write(90);
    rarServo.write(90);
    raeServo.write(90);*/
    
}

void runBody(int chassis, int las, int lar, int lae, int lag, int lat, int lap, int ras, int rar, int rae, int rag, int rat, int rap) {
    //chassisServo.write(chassis + 90);
    
    // Left Arm
    lasServo.write(180 - las);

    if(lar > 90){
      lar = lar * -1;
    }
    larServo.write(lar + 90);
    
    laeServo.write(lae);
    // Left Hand
    
    if(lag > 90) {
      lag = lag * -1;
    }
    lagServo.write(lag + 90);
    
    latServo.write(lat);

    if(lap > 90) {
      lap = lap * -1;
    }
    lapServo.write(lap + 90);
    

    // Right Arm
    rasServo.write(ras);

    if(rar > 90) {
      rar = rar * -1;
    }
    rarServo.write(rar + 90);

    
    raeServo.write(180 - rae);

    // Right Hand
    
    ragServo.write(rag);

    if(rat > 90) {
      rat = rat * -1;
    }
    ratServo.write(rat + 90);
    
    rapServo.write(rap);
    
}
