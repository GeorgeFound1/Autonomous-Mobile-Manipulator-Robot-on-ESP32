#ifndef INTERRUPTS_HPP
#define INTERRUPTS_HPP

// Interrupt handlers for encoders and ultrasonic
void IRAM_ATTR readLeftEncoder();
void IRAM_ATTR readRightEncoder();
void IRAM_ATTR readUltrasonic();

#endif