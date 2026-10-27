#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP

#include <Arduino.h>
#include "pin_modes.hpp"

// Robot position: x, y in cm, angle in degrees
struct Coords {
    double x = 0.0;
    double y = 0.0;
    double angle = 0.0;
};

// Target data from camera
struct TargetData {
    bool detected;
    double distance;
    double angle;
};

class RobotDriver {
public:
    RobotDriver(double startX = 0.0, double startY = 0.0, double startAngle = 0.0) {
        currentCoords.x = startX;
        currentCoords.y = startY;
        currentCoords.angle = startAngle;
    }

    void goToCoords(const double x1, const double y1);
    void searchTarget(float obstacleDistance);

    void update(float obstacleDistance); // Call every loop while moving
    bool isBusy() const { return motionState != MotionState::GOING_STRAIGHT ? false : true; } // True while straight movement is active

    Coords getCoords() const { return currentCoords; }
    float getDistance();
    void stop() { setMotors(0, 0); motionState = MotionState::IDLE; }

    void letTurn(const double angle);
    void startGoStraight(const double distance);
    void goBack(double distanceCm);

private:
    Coords currentCoords;

    enum class MotionState { IDLE, GOING_STRAIGHT };
    MotionState motionState = MotionState::IDLE;

    // State for non-blocking straight movement
    long gsStartLeft = 0;
    long gsStartRight = 0;
    long gsTargetTicks = 0;
    double gsIntegral = 0.0;
    double gsLastError = 0.0;
    unsigned long gsLastTime = 0;

    void updateOdometry();
    void setMotors(int leftSpeed, int rightSpeed);

    void stepGoStraight(float obstacleDistance);
    void goStraightBlocking(const double distance);
};

#endif