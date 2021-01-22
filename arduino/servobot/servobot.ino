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

void log_setup() {
    Serial.begin(115200);
}

void log(char * msg) {
    Serial.println(msg);
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

void print_leg(t_leg_pos * leg, float hip_angle, float knee_angle, float ankle_angle) {
    leg->hip_angle = hip_angle;
    leg->knee_angle = knee_angle;
    leg->ankle_angle = ankle_angle;
    angles_to_xyz(leg);
    xyz_to_angles(leg);
    Serial.print(
        String(hip_angle) +
        "," + String(knee_angle) +
        "," + String(ankle_angle) +
        "," + String(leg->x) +
        "," + String(leg->y) +
        "," + String(leg->z) +
        "," + String(leg->hip_angle) +
        "," + String(leg->knee_angle) +
        "," + String(leg->ankle_angle));
    angles_to_xyz(leg);
    xyz_to_angles(leg);
    Serial.println(
        "," + String(leg->x) +
        "," + String(leg->y) +
        "," + String(leg->z) +
        "," + String(leg->hip_angle) +
        "," + String(leg->knee_angle) +
        "," + String(leg->ankle_angle));
}

void dump_motion_table() {
    t_leg_pos legs[NUM_LEGS];
    t_leg_pos * leg_ptr[NUM_LEGS];
    for (int leg = 0; leg < NUM_LEGS; leg++)
        leg_ptr[leg] = &legs[leg];
    init_legs(leg_ptr);
    int da = 5;
    Serial.println("hip,knee,ankle,x,y,z,hip2,knee2,ankle2,x2,y2,z2,hip3,knee3,ankle3");
    for (int hip_angle = - da; hip_angle <= da; hip_angle += da) {
        for (int knee_angle = -KNEE_DEFLECTION - da; knee_angle <= KNEE_DEFLECTION + da; knee_angle += da) {
            for (int ankle_angle = -ANKLE_DEFLECTION - da; ankle_angle <= ANKLE_DEFLECTION + da; ankle_angle += da) {
                for (int leg = 0; leg < 1; leg++) {
                    print_leg(leg_ptr[leg], hip_angle, knee_angle, ankle_angle);
                    // smooth_move_leg(leg_ptr[leg], 500, 5);
                    // delay(1000);
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
    
    retract();
    delay(100);
    beeps(3);
    dump_motion_table();
}

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
        Serial.print(String(value) + " ");
        if (value >= PPM_LOW && value <= PPM_HIGH) {
            if (value != channel_values[channel - 1]) {
                last_ppm_signal = loop_start;
            }
            channel_values[channel - 1] = value;
        }
    }
    Serial.println();

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
    Serial.print("ride_angle: " + String(ride_angle));
    Serial.print("; spin_rate: " + String(spin_rate));
    Serial.print("; fb: " + String(fb));
    Serial.print("; lr: " + String(lr));
    Serial.print("; walk_speed: " + String(walk_speed));
    Serial.print("; direction: " + String(direction));
    Serial.println();
    if ((loop_start - last_ppm_signal) > PPM_TIMEOUT_MS) {
        // command loss
        Serial.println("command loss");
        channel_values[CHANNEL_ENABLE_DRIVE] = PPM_LOW;
        retract();
        if ((loop_start - last_ppm_signal) > PPM_TIMEOUT_MS * 10) {
            last_ppm_signal = loop_start;
            beep(20);
        }
    }
    if (channel_values[CHANNEL_ENABLE_DRIVE] < PPM_CENTER) {
        // Drive disabled
        delay(250);

    } else if (abs(fb) > DEADZONE_RATIO || abs(lr) > DEADZONE_RATIO) {
        walk(direction, walk_speed, ride_angle);
    } else if (abs(spin_rate) > DEADZONE_RATIO) {
        // Spin in place
        spin(spin_rate, ride_angle);
    } else {
        stand(ride_angle);
    }
}
// */
