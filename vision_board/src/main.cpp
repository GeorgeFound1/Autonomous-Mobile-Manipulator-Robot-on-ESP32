#include "Arduino.h"
#include "esp_camera.h"
#include "camera_config.hpp"
#include "camera_processing.hpp"
#include "protocol.hpp"

// Time between packets to robot
#define SEND_INTERVAL_MS 50

void setup() {
    Serial.begin(115200);
    while(!Serial) { ; }
    
    Serial.println("\n--- ESP32-CAM Test Start ---");

    Serial2.begin(115200, SERIAL_8N1, -1, TX_PIN); 

    if (initCamera()) {
        Serial.println("Camera init SUCCESS!");
    } else {
        Serial.println("Camera init FAILED!");
    }
}

void loop() {
    static unsigned long lastSendTime = 0;
    static float distance = 0.0f;
    static float angle = 0.0f;

    // Capture one camera frame
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return;
    }

    // Detection flag: 1 = target found
    uint8_t statusFlag = 0x00;

    // Detect target and calculate distance/angle
    if (setCoords(fb, &distance, &angle)) {
        statusFlag = 0x01;
        Serial.printf("distance = %f, angle = %f\n", distance, angle);
    } else {
        statusFlag = 0x00;
        distance = 0.0f;
        angle = 0.0f;
        Serial.println("Cannot find the object");
    }

    esp_camera_fb_return(fb);

    // Send packet to robot at fixed rate
    if (millis() - lastSendTime >= SEND_INTERVAL_MS) {
        lastSendTime = millis();

        CameraData packet;
        dataPackage(distance, angle, statusFlag, &packet);

        Serial2.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    }

    delay(10);
}