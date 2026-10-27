#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include <Arduino.h>

// Camera packet structure, packed for binary transfer
#pragma pack(push, 1)
struct CameraData {
    uint8_t startByte;  
    bool detected;  
    float distance;
    float angle;
    uint8_t checksum;
};
#pragma pack(pop)

// Start byte of camera packet
const uint8_t START_MARKER = 0xAA;

// Fill packet with camera data and checksum
void dataPackage(float distance, float angle, bool detected, CameraData *data);
// Calculate XOR checksum
uint8_t calculateChecksum(const uint8_t* data, size_t length);

#endif