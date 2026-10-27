#include "camera_processing.hpp"

// Convert RGB565 camera pixel to RGB888
inline void getRGB(uint8_t *r, uint8_t *g, uint8_t *b, const uint8_t byte1, uint8_t byte2) {

    *r = byte1 & 0xF8;
    *g = (uint8_t)(((byte1 & 0x07) << 5) | ((byte2 & 0xE0) >> 3));
    *b = (uint8_t)((byte2 & 0x1F) << 3);

}

// Estimate distance from target height
float howFar(const int width, const int height) {
    float focalLength = 575.0f; 
    const float realWidth = 1.5f; 
    const float pixelWidth = (float)width;
    const float realHeight = 4.6f;
    const float pixelHeight = (float)height; 

    if (width <= 0 || height <= 0) return 0.0f;
    Serial.printf("width = %d || height = %d\n", width, height);
    
    float distanceX = (focalLength * realWidth) / pixelWidth;
    float distanceY = (focalLength * realHeight) / pixelHeight;

    Serial.printf("dx = %f\n", distanceX);
    Serial.printf("dy = %f\n", distanceY);

    return distanceY;

}

// Convert pixel offset from center to degrees
float whatAngle(const int centerX) {

    const float camFOV = 60.0f; 
    const int frameWidth = 640;   
    const int center = frameWidth / 2;

    const float degPerpixel = camFOV / frameWidth;

    int error = centerX - center;
    float angle = error * degPerpixel;
    return angle;
}

// Check if pixel looks like target color
inline bool isTargetColor(const uint8_t r, const uint8_t g, const uint8_t b) {
    uint16_t sum = r + g + b;
    if (sum < 80) return false;

    float r_norm = (float)r / sum; 
    if (r_norm < 0.52f) return false;

    if (r < g + 45 || r < b + 45) return false;

    if ((uint32_t)g * 100 > (uint32_t)sum * 25) return false;

    return true;
}

// Find target box and calculate smoothed distance/angle
bool setCoords(camera_fb_t *fb, float *smoothedDistance, float *smoothedAngle) {

    if (!fb || !fb->buf || !smoothedDistance || !smoothedAngle) return false;

    uint16_t rowPixels[480] = {0};
    uint16_t colPixels[640] = {0};

    // Search only in the lower part of frame
    for (int y = (fb->height / 3); y < fb->height; y++) {
        for (int x = 0; x < fb->width; x++) {

            int index = (y * fb->width + x) * 2;
            uint8_t byte1 = fb->buf[index];
            uint8_t byte2 = fb->buf[index + 1];
            uint8_t r, g, b, h, s, v;
            getRGB(&r, &g, &b, byte1, byte2);

            if (isTargetColor(r, g, b)) {
                rowPixels[y]++;
                colPixels[x]++;
            }
        }
    }

    // Find bounding box with noise filtering
    const int noiseThresholdY = 10; 
    const int noiseThresholdX = 4;
    const int consecutiveNeeded = 3;

    int minY = -1;
    int consecutiveCount = 0;
    for (int y = 0; y < fb->height; y++) {
        if (rowPixels[y] >= noiseThresholdY) {
            consecutiveCount++;
            if (consecutiveCount >= consecutiveNeeded) {
                minY = y - (consecutiveNeeded - 1);
                break;
            }
        } else {
            consecutiveCount = 0;
        }
    }

    int maxY = -1;
    consecutiveCount = 0;
    for (int y = fb->height - 1; y >= 0; y--) {
        if (rowPixels[y] >= noiseThresholdY) {
            consecutiveCount++;
            if (consecutiveCount >= consecutiveNeeded) {
                maxY = y + (consecutiveNeeded - 1);
                break;
            }
        } else {
            consecutiveCount = 0;
        }
    }

    int minX = -1;
    consecutiveCount = 0;
    for (int x = 0; x < fb->width; x++) {
        if (colPixels[x] >= noiseThresholdX) {
            consecutiveCount++;
            if (consecutiveCount >= consecutiveNeeded) {
                minX = x - (consecutiveNeeded - 1);
                break;
            }
        } else {
            consecutiveCount = 0;
        }
    }

    int maxX = -1;
    consecutiveCount = 0;
    for (int x = fb->width - 1; x >= 0; x--) {
        if (colPixels[x] >= noiseThresholdX) {
            consecutiveCount++;
            if (consecutiveCount >= consecutiveNeeded) {
                maxX = x + (consecutiveNeeded - 1);
                break;
            }
        } else {
            consecutiveCount = 0;
        }
    }

    if (minY == -1 || maxY == -1 || minX == -1 || maxX == -1) return false;

    int width = maxX - minX + 1;
    int height = maxY - minY + 1;


    if (height > 500 || width > 300) {
        return false;
    }

    int boxArea = width * height;

    int actualTargetPixels = 0;
    for (int y = minY; y <= maxY; y++) {
        actualTargetPixels += rowPixels[y];
    }

    float fillRatio = (float)actualTargetPixels / boxArea;

    if (boxArea > 500 && fillRatio < 0.1f) { 
        return false;
    }

    int centerX = (maxX + minX) / 2;

    float distance = howFar(width, height);
    float angle = whatAngle(centerX);
    // Smooth distance and angle to reduce noise
    const float alpha = 0.8f;

    if (*smoothedDistance == 0.0f) {
        *smoothedDistance = distance;
        *smoothedAngle = angle;
    } else {
        *smoothedDistance = alpha * distance + (1.0f - alpha) * (*smoothedDistance);
        *smoothedAngle = alpha * angle + (1.0f - alpha) * (*smoothedAngle);
    }

    return true;
}
