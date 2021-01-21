#include <Wire.h>
#include "PCA9685.h"
#include "servobot.h"
#include "kinematics.c"
#include <PPMReader.h>

PPMReader ppm(INTERRUPT_PIN, NUM_CHANNELS);
unsigned long last_ppm_signal = 0;

PCA9685 driver0;
PCA9685 driver1(B000001);

// TODO: experiment with my servos

// PCA9685 outputs = 12-bit = 4096 steps
// 2.5% of 20ms = 0.5ms ; 12.5% of 20ms = 2.5ms
// 2.5% of 4096 = 102 steps; 12.5% of 4096 = 512 steps
// PCA9685_ServoEval pwmServo(102, 470); // (-90deg, +90deg)
PCA9685_ServoEval pwmServo(102, 470); // (-90deg, +90deg)

unsigned short pwmForAngle(float angle) {
    return pwmServo.pwmForAngle(angle);
}

void set_motor(int driver, int channel, unsigned short pwm) {
    if (driver == 0) {
        driver0.setChannelPWM(
            channel, 
            pwm);
    } else {
        driver1.setChannelPWM(
            channel, 
            pwm);
    }
    // DRIVERS[driver].setChannelPWM(
    //         channel, 
    //         pwmServo.pwmForAngle(angle));
}

unsigned short get_current_value(int driver, int channel) {
    if (driver == 0)
        return driver0.getChannelPWM(channel);
    else
        return driver1.getChannelPWM(channel);
}


void setup() {
    Wire.begin();                 // Wire must be started first
    Wire.setClock(400000);        // Supported baud rates are 100kHz, 400kHz, and 1000kHz
    
    driver0.resetDevices();        // Software resets all PCA9685 devices on Wire line
    driver1.resetDevices();        // Software resets all PCA9685 devices on Wire line
    driver0.init();
    driver1.init();
    driver0.setPWMFrequency(50);   // Set frequency to 50Hz
    driver1.setPWMFrequency(50);   // Set frequency to 50Hz

    // delay(10);
    stand(-90);
    delay(100);
}

void loop() {
    // walk(1.0, 1.0, -45);
    // delay(500);
    // return;
    unsigned long loop_start = millis();
    int channel_values[NUM_CHANNELS];
    // Print latest valid values from all channels
    for (int channel = 1; channel <= NUM_CHANNELS; ++channel) {
        int value = constrain(
            ppm.latestValidChannelValue(channel, 0),
            PPM_LOW, PPM_HIGH);
        // if (value >= PPM_LOW && value <= PPM_HIGH) {
            if (value != channel_values[channel - 1]) {
              last_ppm_signal = loop_start;
            }
            channel_values[channel - 1] = value;
        // }
    }
    float ride_angle = map(
        channel_values[CHANNEL_RIDE_HEIGHT],
        PPM_LOW, PPM_HIGH, -45, 90);
    float spin_rate = map(
            channel_values[CHANNEL_TURN], 
            PPM_LOW, PPM_HIGH, 
            -1.0, 1.0);
    float fb = map(channel_values[CHANNEL_FB], PPM_LOW, PPM_HIGH, -1.0, 1.0);
    float lr = map(channel_values[CHANNEL_LR], PPM_LOW, PPM_HIGH, -1.0, 1.0);
    float walk_speed = sqrt(fb * fb + lr * lr);
    float direction = atan(fb / lr);
    if (channel_values[CHANNEL_ENABLE_DRIVE] < PPM_CENTER || (loop_start - last_ppm_signal) > PPM_TIMEOUT_MS) {
        // Drive disabled or command loss
        delay(250);

    // } else if (abs(fb) > DEADZONE_RATIO || abs(lr) > DEADZONE_RATIO) {
    //     walk(direction, walk_speed, ride_angle);
    } else if (abs(spin_rate) > DEADZONE_RATIO) {
        // Spin in place
        spin(spin_rate, ride_angle);
    } else {
        stand(ride_angle);
    }
}
// */
