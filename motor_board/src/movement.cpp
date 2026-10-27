#include <Arduino.h>
#include <math.h>
#include "movement.hpp"
#include "pin_modes.hpp"
#include "target_search.hpp"

void RobotDriver::setMotors(int leftSpeed, int rightSpeed) {
  // Left Motor
  if (leftSpeed >= 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  ledcWrite(pwmChannelLeft, abs(leftSpeed));

  // Right Motor
  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  ledcWrite(pwmChannelRight, abs(rightSpeed));
}


void RobotDriver::startGoStraight(const double distance) {
    gsStartLeft = leftTicks;
    gsStartRight = rightTicks;
    gsIntegral = 0.0;
    gsLastError = 0.0;
    gsLastTime = millis();
    gsTargetTicks = distance * fromTicksToCM;
    motionState = MotionState::GOING_STRAIGHT;
}

// Drive straight: use encoder difference to correct motors
void RobotDriver::stepGoStraight(float obstacleDistance) {
  static const double Kp = 7.5;
  static const double Ki = 0.09;
  static const double Kd = 0.25;
  static const int maxSpeed = 150;
  static const int minSpeed = 30;


  updateOdometry();

  int baseSpeed = maxSpeed;
  if (currentTarget.detected && currentTarget.distance < TARGET_SLOW_DISTACNE) {
      float t = (currentTarget.distance - TARGET_STOP_DISTANCE) / (TARGET_SLOW_DISTACNE - TARGET_STOP_DISTANCE);
      t = constrain(t, 0.0f, 1.0f);
      baseSpeed = minSpeed + (int)(t * (maxSpeed - minSpeed));
  }


  if (obstacleDistance > 0.0 && obstacleDistance <= 20.0) {
    setMotors(0, 0);
    Serial.println("goStraight: препятствие рядом, остановка");
    motionState = MotionState::IDLE;
    return;
  }

  if (currentTarget.detected && currentTarget.distance <= TARGET_STOP_DISTANCE) {
    setMotors(0, 0);
    Serial.println("goStraight: достигнута дистанция остановки");
    motionState = MotionState::IDLE;
    return;
  }

  if (currentTarget.detected &&
    currentTarget.distance > 0.0 &&
    currentTarget.distance <= TARGET_MAX_TURN_DISTANCE &&
    fabs(currentTarget.angle) > TARGET_SAFE_ANGLE) {
    setMotors(0, 0);
    Serial.printf(
        "goStraight: цель слишком сбоку (%.1f), остановка для коррекции курса\n",
        currentTarget.angle
    );
    motionState = MotionState::IDLE;
    return;
  }

    long currentTicks = (abs(leftTicks - gsStartLeft) + abs(rightTicks - gsStartRight)) / 2;
    if (currentTicks >= gsTargetTicks) {
        setMotors(0, 0);
        Serial.printf("Ticks: LEFT >> %0.2f ||| RIGHT >> %0.2f\n",
                      (double)leftTicks / fromTicksToCM, (double)rightTicks / fromTicksToCM);
        Serial.printf("Приехали в: x = %.2f, y = %.2f, angle = %.02f\n",
                      currentCoords.x, currentCoords.y, currentCoords.angle);
        motionState = MotionState::IDLE;
        return;
    }

    unsigned long currentTime = millis();
    double dt = (currentTime - gsLastTime) / 1000.0;
    if (dt == 0) {
        return; 
    }

    double error = (leftTicks - gsStartLeft) - (rightTicks - gsStartRight);
    gsIntegral += error * dt;
    gsIntegral = constrain(gsIntegral, -500, 500);
    double derivative = (error - gsLastError) / dt;

    double controlOutput = Kp * error + Ki * gsIntegral + Kd * derivative;
    int leftSpeed  = baseSpeed - (int)controlOutput;
    int rightSpeed = baseSpeed + (int)controlOutput;

    leftSpeed  = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    setMotors(leftSpeed, rightSpeed);

    gsLastError = error;
    gsLastTime = currentTime;
}

void RobotDriver::update(float obstacleDistance) {
    if (motionState == MotionState::GOING_STRAIGHT) {
        stepGoStraight(obstacleDistance);
    }
}

void RobotDriver::goStraightBlocking(const double distance) {
    startGoStraight(distance);
    while (motionState == MotionState::GOING_STRAIGHT) {
        float obstacleDistance = getDistance();
        updateTargetData();
        stepGoStraight(obstacleDistance);
    }
}


void RobotDriver::letTurn(const double angle) {

  resetSonarBuffer();

  long startLeft = leftTicks;
  long startRight = rightTicks;

  double Kp = 0.8;
  double Kd = 0.0;
  double Kp_sync = 1.5;

  double error = 0.0;
  double lastError = 0.0;
  double derivative = 0.0;
  int outputSpeed = 0;

  unsigned long lastTime = millis();
  unsigned long currentTime = 0;
  double dt = 0;

  double targetDistance = (abs(angle) / 360.0) * (3.1415 * baseLenght);
  long targetTicks = targetDistance * fromTicksToCM;

  long currentTicks = 0;

  while (abs(currentTicks) < targetTicks) {

    updateOdometry();
    getDistance();
    updateTargetData();

    currentTicks = (abs(leftTicks - startLeft) + abs(rightTicks - startRight)) / 2;
    error = targetTicks - currentTicks;

    currentTime = millis();
    dt = (double)(currentTime - lastTime) / 1000.0;

    if (dt == 0) {
      delay(1);
      continue;
    }
    derivative = (error - lastError) / dt;

    outputSpeed = (int)(Kp * error + Kd * derivative);
    outputSpeed = constrain(outputSpeed, 70, 140);

    double syncError = abs(leftTicks - startLeft) - abs(rightTicks - startRight);

    int leftSpeed = outputSpeed - (int)(Kp_sync * syncError);
    int rightSpeed = outputSpeed + (int)(Kp_sync * syncError);

    leftSpeed = constrain(leftSpeed, 70, 140);
    rightSpeed = constrain(rightSpeed, 70, 140);

    if (angle > 0) {
      setMotors(-leftSpeed, rightSpeed);
    } else {
      setMotors(leftSpeed, -rightSpeed);
    }

    lastError = error;
    lastTime = currentTime;
  }

  setMotors(0, 0);

  resetSonarBuffer();

  Serial.printf("Ticks: LEFT >> %ld ||| RIGHT >> %ld\n", leftTicks, rightTicks);
  delay(10);
}

// Update robot position from encoder ticks
void RobotDriver::updateOdometry() {
    static long lastLeft = 0;
    static long lastRight = 0;

    long currentLeft = leftTicks;
    long currentRight = rightTicks;

    long dLeft = currentLeft - lastLeft;
    long dRight = currentRight - lastRight;

    lastLeft = currentLeft;
    lastRight = currentRight;

    if (dLeft == 0 && dRight == 0) return;

    double dL = (double)dLeft / fromTicksToCM;
    double dR = (double)dRight / fromTicksToCM;

    double dS = (dL + dR) / 2.0;
    double dThetaRad = (dR - dL) / baseLenght;

    double avgAngleRad = currentCoords.angle * (M_PI / 180.0) + (dThetaRad / 2.0);

    currentCoords.x += dS * cos(avgAngleRad);
    currentCoords.y += dS * sin(avgAngleRad);

    currentCoords.angle += dThetaRad * (180.0 / M_PI);
    while (currentCoords.angle > 180.0)  currentCoords.angle -= 360.0;
    while (currentCoords.angle < -180.0) currentCoords.angle += 360.0;
}

// Turn to target point, then drive straight
void RobotDriver::goToCoords(const double x1, const double y1) {
    double x0 = currentCoords.x;
    double y0 = currentCoords.y;
    double angle0 = currentCoords.angle;

    Serial.printf("Начальная точка: x = %.2f, y = %.2f\n", x0, y0);

    double distance = sqrt(pow((x1 - x0), 2) + pow((y1 - y0), 2));
    double angle = atan2(y1 - y0, x1 - x0) * 180 / PI;
    double targetAngle = angle - angle0;
    while (targetAngle > 180) targetAngle -= 360;
    while (targetAngle < -180) targetAngle += 360;

    Serial.printf("Поворачиваем на %.2f угол\n", targetAngle);
    letTurn(targetAngle);

    Serial.println("Едем к цели...");
    startGoStraight(distance); 
}

// Drive backward with wheel sync
void RobotDriver::goBack(double distanceCm) {
    if (distanceCm <= 0) return;

    const int baseSpeed = 100;
    const double Kp_sync = 2.0;

    long startL = leftTicks;
    long startR = rightTicks;
    long targetTicks = round(distanceCm * fromTicksToCM);

    while ((abs(leftTicks - startL) + abs(rightTicks - startR)) / 2 < targetTicks) {
        updateOdometry();

        long dL = abs(leftTicks - startL);
        long dR = abs(rightTicks - startR);
        int corr = (int)(Kp_sync * (dL - dR));

        int speedL = constrain(baseSpeed - corr, 60, 160);
        int speedR = constrain(baseSpeed + corr, 60, 160);

        setMotors(-speedL, -speedR);
        delay(5);
    }

    stop();
}