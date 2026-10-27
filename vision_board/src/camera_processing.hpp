#ifndef PROCESSING_HPP
#define PROCESSING_HPP

#include "Arduino.h"
#include "esp_camera.h"

inline void getRGB(uint8_t *r, uint8_t *g, uint8_t *b, const uint8_t byte1, uint8_t byte2);
// Estimate distance from target size
float howFar(const int width, const int height);
// Estimate angle from target center position
float whatAngle(const int centerX);
// Check if pixel has target color
inline bool isTargetColor(const uint8_t r, const uint8_t g, const uint8_t b);
// Detect target and update distance/angle
bool setCoords (camera_fb_t *fb, float *smoothedDistance, float *smoothedAngle);

#endif