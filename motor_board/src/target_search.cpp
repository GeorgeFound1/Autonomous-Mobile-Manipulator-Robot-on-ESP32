#include <math.h>
#include "target_search.hpp"
#include "movement.hpp"
#include "pin_modes.hpp"

static volatile bool sonarResetRequested = false;

// Request reset of sonar filter
void resetSonarBuffer() {
    sonarResetRequested = true;
}

// Read ultrasonic sensor and filter distance
float RobotDriver::getDistance() {
    static unsigned long lastTrigger = 0;
    static float readings[5] = {400, 400, 400, 400, 400};
    static int readIndex = 0;
    static bool bufferFilled = false;
    static float lastMedian = 400.0;
    
    static bool waitingFirstAfterReset = false;

    if (sonarResetRequested) {
        sonarResetRequested = false;
        
        waitingFirstAfterReset = true;
        
        for (int i = 0; i < 5; i++) {
            readings[i] = 400.0;
        }
        
        readIndex = 0;
        bufferFilled = false;
        lastMedian = 400.0;
        lastTrigger = 0;
    }

    if (millis() - lastTrigger > 60) {
        lastTrigger = millis();
        
        digitalWrite(TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG, LOW);
    }

    if (newReadingAvailable) {
        newReadingAvailable = false;
        
        if (echoDuration > 0) {
            float newDistance = echoDuration / 58.0;
            
            if (newDistance > 400.0) {
                newDistance = 400.0;
            }
            
            if (waitingFirstAfterReset) {
                for (int i = 0; i < 5; i++) {
                    readings[i] = newDistance;
                }
                
                lastMedian = newDistance;
                readIndex = 1;
                bufferFilled = true;
                waitingFirstAfterReset = false;
            } else {
                readings[readIndex] = newDistance;
                readIndex = (readIndex + 1) % 5;
                
                if (readIndex == 0) {
                    bufferFilled = true;
                }
                
                float sorted[5];
                memcpy(sorted, readings, sizeof(sorted));
                
                for (int i = 0; i < 5; i++) {
                    for (int j = i + 1; j < 5; j++) {
                        if (sorted[j] < sorted[i]) {
                            float tmp = sorted[i];
                            sorted[i] = sorted[j];
                            sorted[j] = tmp;
                        }
                    }
                }
                
                lastMedian = sorted[2];
            }
        }
    }

    return lastMedian;
}

// Convert camera target to world coordinates
void calculateCoords(const TargetData& target, const RobotDriver& myRobot, TargetCoords *coords) {
    Coords currentRobotCoords = myRobot.getCoords();
    
    float absoluteAngleRad = (currentRobotCoords.angle + target.angle) * (M_PI / 180.0);

    coords->x = currentRobotCoords.x + target.distance * cos(absoluteAngleRad);
    coords->y = currentRobotCoords.y + target.distance * sin(absoluteAngleRad);

    return;
}

// Target search state machine: drive to wall, then snake movement
void RobotDriver::searchTarget(float obstacleDistance) {

    static SearchState search = DRIVE_TO_WALL_1;
    const double wallSearchDistance = 1000;
    const double snakeStepDistance = 20;
    static int turnDirection = -1;

    // Some search states should not be interrupted by sonar
    bool isDrivingToWall = (search == DRIVE_TO_WALL_1 ||
                        search == TURN_90_AFTER_WALL_1 ||
                        search == DRIVE_TO_WALL_2 ||
                        search == TURN_180_AT_CORNER ||
                        search == SNAKE_MOVE_FORWARD ||
                        search == SNAKE_ROTATE_1 ||
                        search == SNAKE_MOVE_SIDEWARD ||
                        search == SNAKE_ROTATE_2);

    Serial.printf("[SEARCH] state=%d | obstacleDistance=%.1f | isDrivingToWall=%d\n",
                  (int)search, obstacleDistance, isDrivingToWall);

    if (!isDrivingToWall && obstacleDistance > 0.0 && obstacleDistance <= 20.0) {
        Serial.println("[SEARCH] Препятствие близко! Уклонение (letTurn 90)");
        letTurn(90);
        return;
    }

    switch (search)
    {
    case DRIVE_TO_WALL_1:
        Serial.println("[SEARCH] -> DRIVE_TO_WALL_1: старт goStraightBlocking");
        startGoStraight(wallSearchDistance);
        Serial.println("[SEARCH] -> DRIVE_TO_WALL_1 завершён, переход в TURN_90_AFTER_WALL_1");
        search = TURN_90_AFTER_WALL_1;
        break;

    case TURN_90_AFTER_WALL_1:
        Serial.println("[SEARCH] -> TURN_90_AFTER_WALL_1: letTurn(90)");
        letTurn(90);
        Serial.println("[SEARCH] -> переход в DRIVE_TO_WALL_2");
        search = DRIVE_TO_WALL_2;
        break;

    case DRIVE_TO_WALL_2:
        Serial.println("[SEARCH] -> DRIVE_TO_WALL_2: старт goStraightBlocking");
        startGoStraight(wallSearchDistance);
        Serial.println("[SEARCH] -> DRIVE_TO_WALL_2 завершён, переход в TURN_180_AT_CORNER");
        search = TURN_180_AT_CORNER;
        break;

    case TURN_180_AT_CORNER:
        Serial.println("[SEARCH] -> TURN_180_AT_CORNER: letTurn(180)");
        letTurn(180);
        Serial.println("[SEARCH] -> Угол найден, старт змейки");
        search = SNAKE_MOVE_FORWARD;
        break;

    case SNAKE_MOVE_SIDEWARD:
        Serial.println("[SEARCH] -> SNAKE_MOVE_SIDEWARD");
        startGoStraight(snakeStepDistance);
        search = SNAKE_ROTATE_2;
        break;

    case SNAKE_MOVE_FORWARD:
        Serial.println("[SEARCH] -> SNAKE_MOVE_FORWARD");
        startGoStraight(wallSearchDistance);
        search = SNAKE_ROTATE_1;
        break;

    case SNAKE_ROTATE_1:
        Serial.println("[SEARCH] -> SNAKE_ROTATE");
        letTurn(90 * turnDirection);
        search = SNAKE_MOVE_SIDEWARD;
        break;

        case SNAKE_ROTATE_2:
        Serial.println("[SEARCH] -> SNAKE_ROTATE");
        letTurn(90 * turnDirection);
        turnDirection *= -1;
        search = SNAKE_MOVE_FORWARD;
        break;
    }
}

TargetData currentTarget = {false, 0.0, 0.0};
unsigned long lastTargetUpdateTime = 0;

// Update camera target and mark it lost after timeout
void updateTargetData() {
    CameraData packet;
    if (dataUnpackage(&packet)) {
        dataCopy(&packet, &currentTarget);
        lastTargetUpdateTime = millis();
    }
    if (millis() - lastTargetUpdateTime > 500) {
        currentTarget.detected = false;
    }
}

// Normalize angle to -180..180 degrees
double normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}