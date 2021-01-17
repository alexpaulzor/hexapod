#include <Wire.h>
#include "PCA9685.h"
#include "kinematics.c"

#define NUM_LEGS 6
#define NUM_DRIVERS 2

#define CH_HIP_0 0
#define CH_KNEE_0 1
#define CH_ANKLE_0 2
#define CH_HIP_1 3 
#define CH_KNEE_1 4 
#define CH_ANKLE_1 5 
#define CH_HIP_2 6 
#define CH_KNEE_2 7 
#define CH_ANKLE_2 8 

#define CH_HIP_3 0
#define CH_KNEE_3 1
#define CH_ANKLE_3 2
#define CH_HIP_4 3 
#define CH_KNEE_4 4 
#define CH_ANKLE_4 5 
#define CH_HIP_5 6 
#define CH_KNEE_5 7 
#define CH_ANKLE_5 8

const int DRV_CH_HIPS[NUM_LEGS][2] = {
    {0, CH_HIP_0}, 
    {0, CH_HIP_1},
    {0, CH_HIP_2}, 
    {1, CH_HIP_3}, 
    {1, CH_HIP_4},
    {1, CH_HIP_5}
};
const int DRV_CH_KNEES[NUM_LEGS][2] = {
    {0, CH_KNEE_0}, 
    {0, CH_KNEE_1},
    {0, CH_KNEE_2}, 
    {1, CH_KNEE_3}, 
    {1, CH_KNEE_4},
    {1, CH_KNEE_5}
};
const int DRV_CH_ANKLES[NUM_LEGS][2] = {
    {0, CH_ANKLE_0}, 
    {0, CH_ANKLE_1},
    {0, CH_ANKLE_2}, 
    {1, CH_ANKLE_3}, 
    {1, CH_ANKLE_4},
    {1, CH_ANKLE_5}
};

const int LEG_SETS[2][3] = {
    {0, 2, 4},
    {1, 3, 5}
};

PCA9685 driver0;
PCA9685 driver1(B000001);

// TODO: experiment with my servos

// PCA9685 outputs = 12-bit = 4096 steps
// 2.5% of 20ms = 0.5ms ; 12.5% of 20ms = 2.5ms
// 2.5% of 4096 = 102 steps; 12.5% of 4096 = 512 steps
PCA9685_ServoEval pwmServo(102, 470); // (-90deg, +90deg)

void set_motor(int driver, int channel, uint16_t pwm) {
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

void set_angle(int driver, int channel, float angle) {
    set_motor(driver, channel, pwmServo.pwmForAngle(angle));
}

void set_all_angles(int channels[][2], int channels_len, float angle) {
    for (int i = 0; i < channels_len; i++) {
        set_angle(channels[i][0], channels[i][1], angle);
    }
}

void flatten() {
    set_all_angles(DRV_CH_HIPS, NUM_LEGS, 0);
    set_all_angles(DRV_CH_KNEES, NUM_LEGS, 0);
    set_all_angles(DRV_CH_ANKLES, NUM_LEGS, 0);
}

void smooth_move_all(
        long duration_ms, unsigned long interval_ms,
        float leg_angles[NUM_LEGS][3]) {
    // leg_angles = {{hip0, knee0, ankle0}, ...}

    int drivers[3 * NUM_LEGS];
    int channels[3 * NUM_LEGS];
    float angles[3 * NUM_LEGS];
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        int driver = leg / (NUM_LEGS / 2);
        for (int motor = 0; motor < 3; motor++) {
            drivers[3 * leg + motor] = driver;
            channels[3 * leg + motor] = (3 * leg + motor) % (3 * NUM_LEGS);
            angles[3 * leg + motor] = leg_angles[leg][motor];
        } 
    }
    smooth_move_to(
        drivers, channels, angles, 3 * NUM_LEGS,
        duration_ms, interval_ms);
}

void smooth_move_to(
        int drivers[], 
        int channels[], 
        float angles[], 
        int num_channels,
        unsigned long duration_ms,
        unsigned long interval_ms) {

    unsigned long start_time_ms = millis();
    unsigned long last_start_time_ms;
    unsigned long end_time_ms = start_time_ms + duration_ms;
    unsigned long actual_interval_ms = interval_ms;

    while (start_time_ms < end_time_ms) {
        for (int i = 0; i < num_channels; i++) {
            smooth_move_step(
                drivers[i], channels[i], angles[i],
                end_time_ms - millis(),
                actual_interval_ms);
        }
        delay(interval_ms);
        last_start_time_ms = start_time_ms;
        start_time_ms = millis();
        actual_interval_ms = start_time_ms - last_start_time_ms;
    }
}

void smooth_move_step(
        int driver, 
        int channel, 
        float angle, 
        long duration_ms, 
        unsigned long interval_ms) {
    
    uint16_t current_value;  // = DRIVERS[driver].getChannelPWM(channel);
    if (driver == 0)
        current_value = driver0.getChannelPWM(channel);
    else
        current_value = driver1.getChannelPWM(channel);
    
    uint16_t target_value = pwmServo.pwmForAngle(angle);
    uint16_t next_value;
    if (duration_ms <= interval_ms)
        next_value = target_value;
    else
        next_value = map(
            interval_ms, 0, duration_ms, 
            current_value, target_value);
    set_motor(driver, channel, next_value);
}


void dance() {
    // {hip, knee, ankle}
    float dance_moves[][NUM_LEGS][3] = {
        {{0, 0, 0}, 
            {0, 0, 0}, 
            {0, 0, 0}, 
            {0, 0, 0}, 
            {0, 0, 0}, 
            {0, 0, 0}},
        {{0, -45, 0}, 
            {0, -45, 0},  
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 0}},
        {{0, -45, 90}, 
            {0, -45, 90},  
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 0}},
        {{0, -45, 45}, 
            {0, -45, 45}, 
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 00}},
        {{30, -45, 45}, 
            {30, -45, 45}, 
            {0, 0, 0}, 
            {0, 0, 0}, 
            {0, 0, 0}, 
            {0, 0, 0}},
        {{-30, -45, 45}, 
            {-30, -45, 45}, 
            {0, 0, 0}, 
            {0, 0, 0}, 
            {0, 0, 0}, 
            {0, 0, 0}},
        {{0, -45, 45}, 
            {0, -45, 45}, 
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 00}},
        {{0, -45, -45}, 
            {0, -45, 90}, 
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 00}},
        {{0, -45, 90}, 
            {0, -45, -90}, 
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 00}},
        {{0, -45, 0}, 
            {0, -45, 0}, 
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 00}},
        {{0, -45, 90}, 
            {0, -45, 9}, 
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 0}},
        {{0, 0, 90}, 
            {0, 0, 90}, 
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 0}},
        {{0, 45, 45}, 
            {0, 45, 45}, 
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 0}},
        {{0, 45, 45}, 
            {0, -90, -90}, 
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 0}},
        {{0, -90, -90}, 
            {0, -90, -90}, 
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 0}},
        {{0, -90, -90}, 
            {0, 45, 45}, 
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 0}},
        {{0, 45, 45}, 
            {0, 45, 45}, 
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 0}},
        {{0, 45, 45}, 
            {0, 45, 45}, 
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 0}},
        {{0, -45, 0}, 
            {0, -45, 0},  
            {0, 0, 0}, 
            {0, 0, 0},  
            {0, 0, 0},  
            {0, 0, 0}},
        {{0, 0, 0}, 
            {0, 0, 0}, 
            {0, 0, 0}, 
            {0, 0, 0}, 
            {0, 0, 0}, 
            {0, 0, 0}},
    };
    for (int i = 0; i < sizeof(dance_moves)/sizeof(dance_moves[0]); i++) {
        smooth_move_all(1500, 5, dance_moves[i]);
    }
}
/*
void setup() {
    Wire.begin();                 // Wire must be started first
    Wire.setClock(400000);        // Supported baud rates are 100kHz, 400kHz, and 1000kHz
    
    driver0.resetDevices();        // Software resets all PCA9685 devices on Wire line
    driver1.resetDevices();        // Software resets all PCA9685 devices on Wire line
    driver0.init();
    driver1.init();
    driver0.setPWMFrequency(50);   // Set frequency to 50Hz
    driver1.setPWMFrequency(50);   // Set frequency to 50Hz

    delay(10);
    flatten();
    delay(10);
}
void loop() {
    dance();

}
*/
