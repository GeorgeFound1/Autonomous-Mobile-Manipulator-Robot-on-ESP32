#ifndef PIN_MODES_HPP
#define PIN_MODES_HPP

#define RX_PIN 4

// Left motor
const int AIN1 = 17;
const int AIN2 = 16;
const int wirePWMA = 22;
const int encoderLeftA = 32;
const int encoderLeftB = 33;

// Right motor
const int BIN1 = 18;
const int BIN2 = 19;
const int wirePWMB = 23;
const int encoderRightA = 34;
const int encoderRightB = 35;

//Ultsrasonic
const int ECHO = 25;
const int TRIG = 26;


const int pwmChannelLeft = 0; 
const int pwmChannelRight = 1; 
const int pwmFreq = 5000;    
const int pwmResolution = 8; 

//servos
const int servoClaw = 13;
const int servoElbow = 27;
const int servoShoulder = 14;
const int pwmChannelShoulder = 2;
const int pwmChannelElbow = 3;
const int pwmChannelClaw = 4;
const int servoFreq = 50;
const int servoResolution = 14;


const double fromTicksToCM = 51.42; // Encoder ticks per centimeter
const double baseLenght = 15.85; // Distance between wheel centers

extern volatile long leftTicks;
extern volatile long rightTicks;


#endif