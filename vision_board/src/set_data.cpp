#include "protocol.hpp"

// Calculate XOR checksum
uint8_t calculateChecksum(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i]; 
    }
    return crc;
}

// Fill camera packet fields
void dataPackage(float dist, float ang, bool detected, CameraData *data) {
    data->angle = ang;
    data->distance = dist;
    data->detected = detected;
    data->startByte = START_MARKER;
    // Checksum is calculated without the checksum byte
    data->checksum = calculateChecksum((const uint8_t*)data, sizeof(CameraData) - 1);

}