#ifndef MANIPULATOR_HPP
#define MANIPULATOR_HPP

class Manipulator {
public:
    Manipulator();
    void init();
    
    // Grab target using camera distance
    void grabTarget(float distance);
    void resetToBase();  // Lift arm while holding object
    void dropTarget();   // Open claw and park arm

private:
    // Array: [0]=shoulder, [1]=elbow, [2]=claw
    int targetAngles[3]; 

    // Arm geometry in centimeters
    const float L1 = 8.0;          
    const float L_FOREARM = 8.0;  
    const float L_CLAW = 6.5;     
    const float BASE_HEIGHT = 7.0; 


    const float CAM_TO_SHOULDER_OFFSET = 7.1; 
    const float TARGET_GRAB_HEIGHT = 4.0;
    const float REACH_OFFSET = 2.0;

    // Servo calibration: pulse limits for joint angles
    const int SHOULDER_PULSE_MIN = 1500; 
    const int SHOULDER_PULSE_MAX = 500; 
    const int SHOULDER_DEG_MIN   = 0;   
    const int SHOULDER_DEG_MAX   = 87;  


    const int ELBOW_PULSE_MIN = 1000;  
    const int ELBOW_PULSE_MAX = 2000;  
    const int ELBOW_DEG_MIN   = -83;     
    const int ELBOW_DEG_MAX   = -7;     


    const int CLAW_PULSE_OPEN   = 900;   
    const int CLAW_PULSE_CLOSED = 1750;  
    const int CLAW_DEG_OPEN     = 80;    
    const int CLAW_DEG_CLOSED   = 30;    

    const float SHOULDER_PARK_DEG = 90.0; 
    const float ELBOW_PARK_DEG    = 0.0;  


    float currentShoulder;
    float currentElbow;
    float currentClaw;

    // Calculate shoulder and elbow angles from distance
    int* calculateIK(float distance);


    void writeShoulder(float angleDeg);
    void writeElbow(float angleDeg);
    void writeClaw(float angleDeg);


    void moveArmSmooth(float targetShoulder, float targetElbow, int stepDelayMs = 20);
    void moveClawSmooth(float targetClaw, int stepDelayMs = 15);
};

#endif