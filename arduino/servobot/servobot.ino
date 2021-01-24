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
PCA9685_ServoEval pwmServo(MIN_PWM, MAX_PWM); // (-90deg, +90deg)

t_leg_pos legs[NUM_LEGS];
t_leg_pos * leg_ptr[NUM_LEGS];

void log_setup() {
    Serial.begin(115200);
}

void log(char * msg) {
    Serial.println(msg);
}

void log_debug_vals(float leg_foot_ext, float ankle, float hip_foot_angle, float raw_ankle, float raw_knee) {
    Serial.println(
        "leg_foot_ext=" + String(leg_foot_ext) +
        "; ankle=" + String(ankle) +
        "; hfa=" + String(hip_foot_angle) +
        "; raw_ankle=" + String(raw_ankle) +
        "; raw_knee=" + String(raw_knee));
}

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

void print_leg(t_leg_pos * leg) {
    // leg->hip_angle = hip_angle;
    // leg->knee_angle = knee_angle;
    // leg->ankle_angle = ankle_angle;
    // angles_to_xyz(leg);
    // xyz_to_angles(leg);
    // Serial.print(
        // String(hip_angle) +
        // "," + String(knee_angle) +
        // "," + String(ankle_angle) +
        // "," + 
    //     String(leg->x) +
    //     "," + String(leg->y) +
    //     "," + String(leg->z) +
    //     "," + String(leg->hip_angle) +
    //     "," + String(leg->knee_angle) +
    //     "," + String(leg->ankle_angle));
    // angles_to_xyz(leg);
    // xyz_to_angles(leg);
    Serial.println(
        "x=" + String(leg->x) +
        ", y=" + String(leg->y) +
        ", z=" + String(leg->z) +
        ", hip=" + String(leg->hip_angle) +
        ", knee=" + String(leg->knee_angle) +
        ", ankle=" + String(leg->ankle_angle));
}

void dump_motion_table() {
    // t_leg_pos legs[NUM_LEGS];
    // t_leg_pos * leg_ptr[NUM_LEGS];
    // for (int leg = 0; leg < NUM_LEGS; leg++)
    //     leg_ptr[leg] = &legs[leg];
    // init_legs(leg_ptr);
    int da = 15;
    // Serial.println("x,y,z,hip,knee,ankle,");
    for (int hip_angle = - da; hip_angle <= da; hip_angle += da) {
        for (int knee_angle = -KNEE_DEFLECTION; knee_angle <= KNEE_DEFLECTION; knee_angle += da) {
            for (int ankle_angle = -ANKLE_DEFLECTION - da; ankle_angle <= ANKLE_DEFLECTION + da; ankle_angle += da) {
                for (int leg = 0; leg < 1; leg++) {
                    legs[leg].hip_angle = hip_angle;
                    legs[leg].knee_angle = knee_angle;
                    legs[leg].ankle_angle = ankle_angle;
                    angles_to_xyz(leg_ptr[0]);
                    print_leg(leg_ptr[leg]);
                    smooth_move_leg(leg_ptr[leg], 500, 5);
                    delay(500);
                    Serial.println("xyz -> a -> xyz");
                    xyz_to_angles(leg_ptr[0]);
                    angles_to_xyz(leg_ptr[0]);
                    print_leg(leg_ptr[0]);
                    // smooth_move_leg(leg_ptr[0], 500, 5);
                    // beeps(3);
                    // delay(500);
                }
            }
        }
    }
    delay(10000);
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
    pinMode(BUZZER_PIN, OUTPUT);
    log_setup();
    for (int leg = 0; leg < NUM_LEGS; leg++)
        leg_ptr[leg] = &legs[leg];
    init_legs(leg_ptr);
    retract(leg_ptr);
    // angles_to_xyz(leg_ptr[0]);
    // print_leg(leg_ptr[0]);
    beeps(3);
    // delay(100);
    // test_leg(250, 0, 0);
    // test_leg(250, 0, 0);
    // test_leg(200, 0, 0);
    // test_leg(200, 0, 50);
    // test_leg(200, 0, -50);
    // dump_motion_table();
    
    // testleg2(0, -60, -60);
    // beep(2);
    // delay(5000);
    // testleg2(0, -22.5, 0);
    // testleg2(10, -30, -30);
    // beeps(7);
    // delay(5000);
}
/*
void testleg2(float hip, float knee, float ankle) {
    legs[0].hip_angle = hip;
    legs[0].knee_angle = knee;
    legs[0].ankle_angle = ankle;
    Serial.println(
        "-> (hip=" + String(hip) + 
        ", knee=" + String(knee) + 
        ", ankle=" + String(ankle) + ") -> xyz:");
    angles_to_xyz(leg_ptr[0]);
    print_leg(leg_ptr[0]);
    smooth_move_leg(leg_ptr[0], 500, 5);
    beeps(3);
    delay(5000);
    test_leg(legs[0].x, legs[0].y, legs[0].z);
}

void test_leg(float x, float y, float z) {
    legs[0].y = y;
    legs[0].z = z;
    legs[0].x = x;
    Serial.println(
        "-> (x=" + String(x) + 
        ", y=" + String(y) + 
        ", z=" + String(z) + ") -> angles:");
    xyz_to_angles(leg_ptr[0]);
    print_leg(leg_ptr[0]);
    smooth_move_leg(leg_ptr[0], 500, 5);
    beeps(4);
    delay(5000);
    Serial.println("angles -> xyz");
    angles_to_xyz(leg_ptr[0]);
    print_leg(leg_ptr[0]);
    // smooth_move_leg(leg_ptr[0], 500, 5);
    // beeps(5);
    // delay(5000);
    Serial.println("xyz -> angles");
    xyz_to_angles(leg_ptr[0]);
    print_leg(leg_ptr[0]);
    smooth_move_leg(leg_ptr[0], 500, 5);
    beeps(6);
    delay(5000);
}*/

void loop() {
    // 
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
        // Serial.print(String(value) + " ");
        if (value >= PPM_LOW && value <= PPM_HIGH) {
            if (value != channel_values[channel - 1]) {
                last_ppm_signal = loop_start;
            }
            channel_values[channel - 1] = value;
        }
    }
    
    float ride_angle = map(
        channel_values[CHANNEL_RIDE_HEIGHT],
        PPM_LOW, PPM_HIGH, -45, 90);
    float spin_rate = mapf(
            channel_values[CHANNEL_TURN], 
            PPM_LOW, PPM_HIGH, 
            -1.0, 1.0);
    float fb = mapf(channel_values[CHANNEL_FB], PPM_LOW, PPM_HIGH, -1.0, 1.0);
    float lr = mapf(channel_values[CHANNEL_LR], PPM_LOW, PPM_HIGH, -1.0, 1.0);
    float walk_speed = sqrt(fb * fb + lr * lr);
    float direction = atan(fb / lr);
    // Serial.print("ride_angle: " + String(ride_angle));
    // Serial.print("; spin_rate: " + String(spin_rate));
    // Serial.print("; fb: " + String(fb));
    // Serial.print("; lr: " + String(lr));
    // Serial.print("; walk_speed: " + String(walk_speed));
    // Serial.print("; direction: " + String(direction));
    // Serial.println();
    if ((loop_start - last_ppm_signal) > PPM_TIMEOUT_MS) {
        // command loss
        Serial.println("command loss");
        channel_values[CHANNEL_ENABLE_DRIVE] = PPM_LOW;
        retract(leg_ptr);
        if ((loop_start - last_ppm_signal) > PPM_TIMEOUT_MS * 10) {
            last_ppm_signal = loop_start;
            beep(20);
        }
    }
    if (channel_values[CHANNEL_ENABLE_DRIVE] < PPM_CENTER) {
        // Drive disabled
        delay(250);

    } else if (abs(fb) > DEADZONE_RATIO || abs(lr) > DEADZONE_RATIO) {
        walk(leg_ptr, direction, walk_speed, ride_angle);
    } else if (abs(spin_rate) > DEADZONE_RATIO) {
        // Spin in place
        // TODO: port to use t_leg_pos
        spin(spin_rate, ride_angle);
    } else {
        stand(leg_ptr, ride_angle);
    }
}
// */
