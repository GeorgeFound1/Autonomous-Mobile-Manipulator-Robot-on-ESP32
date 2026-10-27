#ifndef TARGET_SEARCH_HPP
#define TARGET_SEARCH_HPP

#include "movement.hpp"
#include "protocol.hpp"

// Target position in world coordinates
struct TargetCoords {

    double x;
    double y;
};

// Search algorithm states
enum SearchState { 
    DRIVE_TO_WALL_1,
    TURN_90_AFTER_WALL_1,
    DRIVE_TO_WALL_2,
    TURN_180_AT_CORNER,
    SNAKE_MOVE_FORWARD,
    SNAKE_MOVE_SIDEWARD,
    SNAKE_ROTATE_1,
    SNAKE_ROTATE_2

};

// Ultrasonic variables updated by interrupts
extern volatile unsigned long echoDuration;
extern volatile bool newReadingAvailable;
extern volatile unsigned long echoStartTime;

// Thresholds for target approach
const float TARGET_SLOW_DISTACNE = 100.0;
const float TARGET_STOP_DISTANCE = 10.0;
const float TARGET_SAFE_ANGLE = 3.0;
const float TURN_CORRECTION_FACTOR = 0.4;
const float TARGET_MAX_TURN_DISTANCE = 70.0;
const unsigned long ANGLE_DEBOUNCE_MS = 200;
const unsigned long CAMERA_SETTLE_TIMEOUT = 700;

// Last known target from camera
extern TargetData currentTarget;
// Time of last target update
extern unsigned long lastTargetUpdateTime;

void updateTargetData();
void resetSonarBuffer();

void calculateCoords(const TargetData& target, const RobotDriver& myRobot, TargetCoords *coords);
double normalizeAngle(double angle); 

#endif
