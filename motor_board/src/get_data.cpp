#include "protocol.hpp"
#include "target_search.hpp"

uint8_t calculateChecksum(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i]; 
    }
    return crc;
}

void dataCopy(CameraData *data, TargetData *target) {

    target->detected = data->detected;
    target->distance = (double)data->distance;
    target->angle = (double)data->angle;

    return;
}

// Read and validate camera packet
bool dataUnpackage(CameraData *packet) {

    while (Serial2.available() > 0 && Serial2.peek() != START_MARKER) {
        Serial2.read();
    }

    if (Serial2.available() >= sizeof(CameraData)) {
        uint8_t buffer[sizeof(CameraData)];
        
        size_t bytesRead = Serial2.readBytes(buffer, sizeof(CameraData));
        
        if (bytesRead == sizeof(CameraData)) {
            uint8_t crc = calculateChecksum(buffer, sizeof(CameraData) - 1);
            
            if (crc == buffer[sizeof(CameraData) - 1] && (buffer[0] == START_MARKER)) {
                memcpy(packet, buffer, sizeof(CameraData));
                return true;
            }
        }
    }
    return false;
}