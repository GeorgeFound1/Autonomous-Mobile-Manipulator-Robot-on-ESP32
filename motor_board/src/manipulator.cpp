#include "manipulator.hpp"
#include "pin_modes.hpp" 
#include <Arduino.h>

Manipulator::Manipulator() {
    currentShoulder = SHOULDER_PARK_DEG;
    currentElbow    = ELBOW_PARK_DEG;
    currentClaw     = CLAW_DEG_OPEN;
}

void Manipulator::init() {
    ledcSetup(pwmChannelShoulder, servoFreq, servoResolution);
    ledcSetup(pwmChannelElbow, servoFreq, servoResolution);
    ledcSetup(pwmChannelClaw, servoFreq, servoResolution);

    ledcAttachPin(servoShoulder, pwmChannelShoulder);
    ledcAttachPin(servoElbow, pwmChannelElbow);
    ledcAttachPin(servoClaw, pwmChannelClaw);

    currentShoulder = constrain(SHOULDER_PARK_DEG, SHOULDER_DEG_MIN, SHOULDER_DEG_MAX);
    currentElbow    = constrain(ELBOW_PARK_DEG, ELBOW_DEG_MIN, ELBOW_DEG_MAX);

    writeShoulder(currentShoulder);
    writeElbow(currentElbow);
    writeClaw(currentClaw);
}

// Convert angles to servo PWM signals

void Manipulator::writeShoulder(float angleDeg) {
    angleDeg = constrain(angleDeg, SHOULDER_DEG_MIN, SHOULDER_DEG_MAX);
    
    int pulseUs = map(round(angleDeg), 
                      SHOULDER_DEG_MIN, SHOULDER_DEG_MAX, 
                      SHOULDER_PULSE_MIN, SHOULDER_PULSE_MAX);

    uint32_t duty = ((uint32_t)pulseUs * 16383UL) / 20000UL;
    ledcWrite(pwmChannelShoulder, duty);
}

void Manipulator::writeElbow(float angleDeg) {

    angleDeg = constrain(angleDeg, ELBOW_DEG_MIN, ELBOW_DEG_MAX);
    
    int pulseUs = map(round(angleDeg), 
                      ELBOW_DEG_MIN, ELBOW_DEG_MAX, 
                      ELBOW_PULSE_MIN, ELBOW_PULSE_MAX);
    
    uint32_t duty = ((uint32_t)pulseUs * 16383UL) / 20000UL;
    ledcWrite(pwmChannelElbow, duty);
}

void Manipulator::writeClaw(float angleDeg) {

    int minDeg = min(CLAW_DEG_OPEN, CLAW_DEG_CLOSED);
    int maxDeg = max(CLAW_DEG_OPEN, CLAW_DEG_CLOSED);
    angleDeg = constrain(angleDeg, minDeg, maxDeg);

    int pulseUs = map(round(angleDeg), 
                      CLAW_DEG_OPEN, CLAW_DEG_CLOSED, 
                      CLAW_PULSE_OPEN, CLAW_PULSE_CLOSED);

    uint32_t duty = ((uint32_t)pulseUs * 16383UL) / 20000UL;
    ledcWrite(pwmChannelClaw, duty);
}



void Manipulator::moveArmSmooth(float targetShoulder, float targetElbow, int stepDelayMs) {

    targetShoulder = constrain(targetShoulder, SHOULDER_DEG_MIN, SHOULDER_DEG_MAX);
    targetElbow    = constrain(targetElbow, ELBOW_DEG_MIN, ELBOW_DEG_MAX);

    float startS = currentShoulder;
    float startE = currentElbow;
    
    int diffS = abs(round(targetShoulder - startS));
    int diffE = abs(round(targetElbow - startE));
    int steps = max(diffS, diffE);
    
    if (steps == 0) return;

    for (int i = 1; i <= steps; i++) {
        currentShoulder = startS + (targetShoulder - startS) * ((float)i / steps);
        currentElbow    = startE + (targetElbow - startE) * ((float)i / steps);
        
        writeShoulder(currentShoulder);
        writeElbow(currentElbow);
        delay(stepDelayMs);
    }
}

void Manipulator::moveClawSmooth(float targetClaw, int stepDelayMs) {
    int minDeg = min(CLAW_DEG_OPEN, CLAW_DEG_CLOSED);
    int maxDeg = max(CLAW_DEG_OPEN, CLAW_DEG_CLOSED);
    targetClaw = constrain(targetClaw, minDeg, maxDeg);

    float startC = currentClaw;
    int steps = abs(round(targetClaw - startC));
    
    if (steps == 0) return;

    for (int i = 1; i <= steps; i++) {
        currentClaw = startC + (targetClaw - startC) * ((float)i / steps);
        writeClaw(currentClaw);
        delay(stepDelayMs);
    }
}


// Calculate arm angles to reach the target

int* Manipulator::calculateIK(float distance) {

    float realDistance = distance + CAM_TO_SHOULDER_OFFSET + REACH_OFFSET;

    float y = -BASE_HEIGHT + TARGET_GRAB_HEIGHT;

    // Wrist point is target point minus claw length
    float wrist_x = realDistance - L_CLAW;
    float wrist_y = y;

    float r = sqrt(wrist_x * wrist_x + wrist_y * wrist_y);
    float max_reach = L1 + L_FOREARM - 0.1f;
    float min_reach = fabs(L1 - L_FOREARM) + 0.1f;

    if (r > max_reach) {
        Serial.printf("[IK] ВНИМАНИЕ: Цель слишком далеко (r=%.1f > %.1f)! Подъедьте ближе или уменьшите REACH_OFFSET.\n", r, max_reach);
        float scale = max_reach / r;
        wrist_x *= scale;
        wrist_y *= scale;
        r = max_reach;
    } else if (r < min_reach) {
        float scale = min_reach / r;
        wrist_x *= scale;
        wrist_y *= scale;
        r = min_reach;
    }

    float gamma = atan2(wrist_y, wrist_x);

    float cosAlpha = (r * r + L1 * L1 - L_FOREARM * L_FOREARM) / (2.0f * r * L1);
    cosAlpha = constrain(cosAlpha, -1.0f, 1.0f);
    float alpha = acos(cosAlpha);

    float cosBeta = (r * r + L_FOREARM * L_FOREARM - L1 * L1) / (2.0f * r * L_FOREARM);
    cosBeta = constrain(cosBeta, -1.0f, 1.0f);
    float beta = acos(cosBeta);

    float rawS = (gamma + alpha) * 180.0f / PI;
    float rawE = (gamma - beta) * 180.0f / PI;

    // If shoulder limit is reached, correct elbow to keep grab height
    if (rawS < SHOULDER_DEG_MIN) {
        Serial.printf("[IK] ПЛЕЧО упёрлось в нижний лимит (%d°) -> корректируем локоть по высоте\n", SHOULDER_DEG_MIN);
        rawS = SHOULDER_DEG_MIN;
        float sRad = rawS * PI / 180.0f;
        float elbow_y = L1 * sin(sRad);
        float sinElbow = (y - elbow_y) / L_FOREARM;
        sinElbow = constrain(sinElbow, -1.0f, 1.0f);
        rawE = asin(sinElbow) * 180.0f / PI;
    }

    Serial.printf("[IK] dist=%.1f realDist=%.1f wrist=(%.1f, %.1f) -> плечо=%.0f° локоть=%.0f° | лимиты: плечо %d..%d, локоть %d..%d\n",
                  distance, realDistance, wrist_x, wrist_y, rawS, rawE,
                  SHOULDER_DEG_MIN, SHOULDER_DEG_MAX, ELBOW_DEG_MIN, ELBOW_DEG_MAX);

    if (rawS > SHOULDER_DEG_MAX) Serial.println("[IK] ПЛЕЧО упёрлось в верхний лимит");
    if (rawE < ELBOW_DEG_MIN || rawE > ELBOW_DEG_MAX) Serial.println("[IK] ЛОКОТЬ упёрся в лимит");

    targetAngles[0] = round(constrain(rawS, SHOULDER_DEG_MIN, SHOULDER_DEG_MAX));
    targetAngles[1] = round(constrain(rawE, ELBOW_DEG_MIN, ELBOW_DEG_MAX));
    targetAngles[2] = CLAW_DEG_CLOSED;

    return targetAngles;
}

// Grab sequence: move arm, close claw, then lift
void Manipulator::grabTarget(float distance) {

    moveClawSmooth(CLAW_DEG_OPEN, 10); 

    int* angles = calculateIK(distance);

    moveArmSmooth(angles[0], angles[1], 20);

    delay(300); 

    moveClawSmooth(angles[2], 15);
}

// Park arm; claw keeps holding object
void Manipulator::resetToBase() {

    moveArmSmooth(SHOULDER_PARK_DEG, ELBOW_PARK_DEG, 20); 
}

void Manipulator::dropTarget() {
    
    moveClawSmooth(CLAW_DEG_OPEN, 15); 
    delay(500); 
    resetToBase(); 
}
