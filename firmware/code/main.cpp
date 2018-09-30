#include "mbed.h"
#include "pins.h"
#include "motor.h"
#include "USBSerial.h"
#include "RFManager.h"

USBSerial serial;

Serial pc(USBTX, USBRX);

DigitalOut led1(LED_1);
DigitalOut led2(LED_2);
DigitalOut led3(LED_3);

static const int NUMBER_OF_MOTORS = 3;

PwmOut m3(M3_PWM);

Motor motor0(M0_PWM, M0_DIR1, M0_DIR2, M0_ENCA, M0_ENCB);
Motor motor1(M1_PWM, M1_DIR1, M1_DIR2, M1_ENCA, M1_ENCB);
Motor motor2(M2_PWM, M2_DIR1, M2_DIR2, M2_ENCA, M2_ENCB);
//Motor motor3(M3_PWM, M3_DIR1, M3_DIR2, M3_ENCA, M3_ENCB);

Motor * motors[NUMBER_OF_MOTORS] = {
    &motor0, &motor1, &motor2
};

RFManager rfModule(COM1_TX, COM1_RX);

void serialInterrupt();
void parseCommand(char *command);

Ticker pidTicker;
unsigned int pidTickerCount = 0;
static const float PID_FREQ = 60;

char buf[32];
int serialCount = 0;
bool serialData = false;

bool failSafeEnabled = true;
int failSafeCount = 0;
int failSafeLimit = 60;

void pidTick() {
    for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
        motors[i]->pid();
    }

    if (pidTickerCount++ % 60 == 0) {
        led1 = !led1;
    }

    failSafeCount++;
    
    if (failSafeCount == failSafeLimit) {
        failSafeCount = 0;
        
        if (failSafeEnabled) {
            for (int i = 0; i < NUMBER_OF_MOTORS; ++i) {
                motors[i]->setSpeed(0);
            }

            m3.pulsewidth_us(800);
        }
    }
}

int main() {
    pidTicker.attach(pidTick, 1 / PID_FREQ);
    //serial.attach(&serialInterrupt);

    // Dribbler motor
    //m3.period_us(4000);
    m3.pulsewidth_us(800);

    while (1) {
        if (rfModule.readable()) {
            serial.printf("<ref:%s>\n", rfModule.read());
        }

        rfModule.update();

        if (serial.readable()) {
            buf[serialCount] = serial.getc();

            if (buf[serialCount] == '\n') {
                parseCommand(buf);
                serialCount = 0;
                memset(buf, 0, 32);
            } else {
                serialCount++;
            }
        }
    }
}

void parseCommand(char *buffer) {
    failSafeCount = 0;

    char *cmd = strtok(buffer, ":");

    // buffer == "sd:14:16:10:30"
    if (strncmp(cmd, "sd", 2) == 0) {
        for (int i = 0; i < NUMBER_OF_MOTORS; ++i) {
            motors[i]->setSpeed(atoi(strtok(NULL, ":")));
        }

        serial.printf("<gs:%d:%d:%d>\n",
            motors[0]->getSpeed(),
            motors[1]->getSpeed(),
            motors[2]->getSpeed()
         );
    }

    if (strncmp(cmd, "d", 1) == 0) {
        int pulsewidth = atoi(buffer + 2);
        
        if (pulsewidth < 800) {
            pulsewidth = 800;
        } else if (pulsewidth > 2100) {
           pulsewidth = 2100;
        }
        
        m3.pulsewidth_us(pulsewidth);        
    }

    if (strncmp(cmd, "gs", 2) == 0) {
        serial.printf("<gs:%d:%d:%d>\n",
            motors[0]->getSpeed(),
            motors[1]->getSpeed(),
            motors[2]->getSpeed()
        );
    }

    if (strncmp(cmd, "rf", 2) == 0) {
        rfModule.send(buffer + 3);
    }

    if (strncmp(cmd, "fs", 1) == 0) {
        failSafeEnabled = buffer[3] != '0';
    }
}
