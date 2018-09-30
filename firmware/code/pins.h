#ifndef PINS_H_
#define PINS_H_
#include "mbed.h"

// MOTOR0 (J1)
#define M0_PWM P2_0
#define M0_DIR1 P4_29
#define M0_DIR2 P4_28
#define M0_ENCA P0_7
#define M0_ENCB P0_6

// MOTOR1 (J4)
#define M1_PWM P2_1
#define M1_DIR1 P0_9
#define M1_DIR2 P0_8
#define M1_ENCA P2_6
#define M1_ENCB P2_7

// MOTOR2 (J5)
#define M2_PWM P2_2
#define M2_DIR1 P0_15
#define M2_DIR2 P0_16
#define M2_ENCA P0_17
#define M2_ENCB P0_18

// MOTOR3 (J6)
#define M3_PWM P2_3
#define M3_DIR1 P0_19
#define M3_DIR2 P0_20
#define M3_ENCA P0_21
#define M3_ENCB P0_22

// LEDS
#define LED_1 P1_18
#define LED_2 P1_19
#define LED_3 P1_22

// COM1 (J9)
#define COM1_TX P0_0
#define COM1_RX P0_1

// COM2 (J11)
#define COM2_TX P0_10
#define COM2_RX P0_11

// SPI (J10)
#define SPI_MOSI P1_24
#define SPI_MISO P1_23
#define SPI_SSEL P1_21
#define SPI_SCK P1_20

// ISOLATED_IO (J8)
#define ISO_PWM5 P2_4
#define ISO_PWM6 P2_5
#define ISO_OUT P0_4
#define ISO_IN P0_5

#endif
