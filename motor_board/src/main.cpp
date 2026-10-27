#include <Arduino.h>
#include "movement.hpp"
#include "pin_modes.hpp"
#include "interrupts.hpp"
#include "protocol.hpp"
#include "target_search.hpp"
#include "manipulator.hpp"   

volatile long leftTicks = 0;
volatile long rightTicks = 0;

volatile unsigned long echoStartTime = 0;
volatile unsigned long echoDuration = 0;
volatile bool newReadingAvailable = false;

#define TIME_TO_GET_PACKET 500
#define GRAB_SETTLE_MS 5000  

#define REALIGN_GAIN   0.6   
#define REALIGN_TOL    3.0   
#define MAX_REALIGN    3     

Manipulator arm;             

void setup() {
  Serial.begin(115200);

  Serial2.begin(115200, SERIAL_8N1, RX_PIN, -1);
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  ledcSetup(pwmChannelLeft, pwmFreq, pwmResolution);
  ledcSetup(pwmChannelRight, pwmFreq, pwmResolution);
  
  ledcAttachPin(wirePWMA, pwmChannelLeft);
  ledcAttachPin(wirePWMB, pwmChannelRight);

  pinMode(encoderLeftA, INPUT_PULLUP);
  pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP);
  pinMode(encoderRightB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderLeftA), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), readRightEncoder, RISING);

  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO), readUltrasonic, CHANGE);

  arm.init();             \

  delay(10000);
}

void loop() {

    //arm.calibrationLoop(); return;
    static RobotDriver myRobot;
    static TargetData lastTarget = {false, 0.0, 0.0};
    static bool finalTurnDone = false;
    static unsigned long lastSeenTime = 0;

    static bool isCorrectingAngle = false;
    static unsigned long correctionStartTime = 0;

    // [ARM]
    static bool grabPending = false;          
    static bool grabDone = false;            
    static unsigned long grabWaitStart = 0;
    static bool isReturning = false;         

    static bool carryingObject = false;     

    static int realignCount = 0;             
    static bool movedBack = false;            
    static double estAngleAfterBack = 0.0;    

    const unsigned long LOST_GRACE_PERIOD = 2000;

    // Ignore sonar while carrying an object
    float obstacleDistance = carryingObject ? 400.0f : myRobot.getDistance();
    updateTargetData();

    static unsigned long lastLog = 0;
    if (millis() - lastLog > 200) {
        lastLog = millis();
        Serial.printf(
            "[LOG] Sonar: %.1f | Cam: %d | Dist: %.1f | Ang: %.1f\n",
            obstacleDistance,
            currentTarget.detected,
            currentTarget.distance,
            currentTarget.angle
        );
    }

    if (myRobot.isBusy()) {
        myRobot.update(obstacleDistance);
        return;
    }

    if (isReturning) {
        Serial.println("-> БАЗА ДОСТИГНУТА! Остановка.");
        myRobot.stop();
        isReturning = false;
        grabDone = true;
        return;
    }

    if (grabDone) {
        myRobot.stop();
        return;
    }

    bool targetInWorkZone = currentTarget.detected &&
                            currentTarget.distance > 0.0 &&
                            currentTarget.distance <= TARGET_MAX_TURN_DISTANCE;

    if (targetInWorkZone) {
        lastSeenTime = millis();
        lastTarget = currentTarget;

        if (currentTarget.distance > TARGET_STOP_DISTANCE + 5.0) {
            finalTurnDone = false;
            grabPending = false;
            realignCount = 0;   
            movedBack = false;   
        }
    }

    // Wait before grab, then check distance and align
    if (grabPending) {
        myRobot.stop();
        lastSeenTime = millis();

        if (millis() - grabWaitStart < GRAB_SETTLE_MS) {
            return;
        }
        // If too close, move back to safe distance
        if (lastTarget.distance > 0.0 && lastTarget.distance < 9.5) {
            float backDist = 10.0 - lastTarget.distance;
            Serial.printf("-> Слишком близко (%.1f см < 9.5 см). Отъезд назад на %.1f см до 10.0 см...\n",
                          lastTarget.distance, backDist);


            double a0 = lastTarget.angle * M_PI / 180.0;
            estAngleAfterBack = atan2(lastTarget.distance * tan(a0), lastTarget.distance + backDist) * 180.0 / M_PI;
            movedBack = true;
            realignCount = 0; 

            myRobot.goBack(backDist);

            while (Serial2.available()) {
                Serial2.read();
            }
            currentTarget.detected = false;
            lastTarget.distance = 10.0; 
            grabWaitStart = millis();
            return;
        }

        // Small turn to aim at target before grab
        if (realignCount < MAX_REALIGN) {
            double ang = currentTarget.detected ? currentTarget.angle
                                                : ((movedBack && realignCount == 0) ? estAngleAfterBack : lastTarget.angle);

            if (fabs(ang) > REALIGN_TOL) {
                realignCount++;
                double turnAngle = ang * REALIGN_GAIN;
                Serial.printf("-> ДОВОРОТ ПЕРЕД ЗАХВАТОМ %d/%d: угол %.1f -> поворот на %.1f град (%s)\n",
                              realignCount, MAX_REALIGN, ang, turnAngle,
                              currentTarget.detected ? "по камере" : "расчётный");

                myRobot.letTurn(turnAngle);

                while (Serial2.available()) {
                    Serial2.read();
                }
                currentTarget.detected = false;
                grabWaitStart = millis();
                return;
            }
        }

        grabPending = false;
        realignCount = 0;
        movedBack = false;

        Serial.printf("-> ЗАХВАТ: дистанция до цели %.1f см\n", lastTarget.distance);
        // Grab object, lift arm, and return to base
        arm.grabTarget(lastTarget.distance);
        carryingObject = true;  
        
        Serial.println("-> ПОДЪЕМ манипулятора с целью...");
        arm.resetToBase();
        
        Serial.println("-> ВОЗВРАЩЕНИЕ НА БАЗУ (0.0, 0.0)...");
        myRobot.goToCoords(0.0, 0.0);
        
        isReturning = true;
        return;
    }

    if (isCorrectingAngle) {
        if (millis() - correctionStartTime < 1000) {
            myRobot.stop();
            return;
        } else {
            isCorrectingAngle = false;
        }
    }

    if (!targetInWorkZone && millis() - lastSeenTime > LOST_GRACE_PERIOD) {
        finalTurnDone = false;
        myRobot.searchTarget(obstacleDistance);
        return;
    }

    if (targetInWorkZone && currentTarget.distance <= TARGET_STOP_DISTANCE) {
        myRobot.stop();

        if (!finalTurnDone) {
            finalTurnDone = true;

            Serial.printf(
                "-> ФИНАЛЬНЫЙ ПОВОРОТ на %.1f град\n",
                currentTarget.angle
            );

            myRobot.letTurn(currentTarget.angle);
            lastSeenTime = millis();

            grabPending = true;         
            realignCount = 0;        
            movedBack = false;       
            grabWaitStart = millis();
        }

        return;
    }

    if (!targetInWorkZone && lastTarget.detected && !finalTurnDone) {
        if (lastTarget.distance <= TARGET_STOP_DISTANCE + 10.0 &&
            millis() - lastSeenTime < 1000) {

            finalTurnDone = true;

            Serial.printf(
                "-> Цель потеряна у финиша, доворот на %.1f град\n",
                lastTarget.angle
            );

            myRobot.letTurn(lastTarget.angle);
            lastSeenTime = millis();

            grabPending = true;      
            realignCount = 0;       
            movedBack = false;      
            grabWaitStart = millis();

            return;
        }
    }

    if (targetInWorkZone &&
        currentTarget.distance > TARGET_STOP_DISTANCE &&
        fabs(currentTarget.angle) > TARGET_SAFE_ANGLE) {

        myRobot.stop();

        double turnAngle = currentTarget.angle * 0.7;

        Serial.printf(
            "-> Коррекция курса: поворот на %.1f град (цель была %.1f)\n",
            turnAngle,
            currentTarget.angle
        );

        myRobot.letTurn(turnAngle);

        isCorrectingAngle = true;
        correctionStartTime = millis();
        lastSeenTime = millis();

        return;
    }

    if (targetInWorkZone && currentTarget.distance > TARGET_STOP_DISTANCE) {

        if (obstacleDistance > 0.0 && obstacleDistance <= 25.0) {
            myRobot.stop();
            return;
        }

        double d = currentTarget.distance - TARGET_STOP_DISTANCE;

        if (d > 20.0) {
            d = 20.0;
        }

        if (d > 2.0) {
            Serial.printf("-> Подъезд к цели, участок %.1f см\n", d);
            myRobot.startGoStraight(d);
        }

        return;
    }

    if (!targetInWorkZone && millis() - lastSeenTime < LOST_GRACE_PERIOD) {
        myRobot.stop();
        return;
    }
}