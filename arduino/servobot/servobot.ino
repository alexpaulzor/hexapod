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
        "r=" + String(leg->rotation) +
        ", x=" + String(leg->x) +
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

void xyz_to_angles(t_leg_pos * leg) {
    float dx = leg->x - BODY_PIVOT_R * cos(DEG2RAD * (leg->rotation));
    float dy = leg->y - BODY_PIVOT_R * sin(DEG2RAD * (leg->rotation));
    float dz = leg->z - HIP_DZ;
    float raw_hip = atan(dy / dx) * RAD2DEG - leg->rotation;
    if (raw_hip < -90)
        raw_hip += 360;
    if (raw_hip > 90)
        raw_hip -= 180;
    Serial.println("Raw hip_angle=" + String(raw_hip) + "; r=" + String(leg->rotation));
    leg->hip_angle = constrain(
        (raw_hip),
        -HIP_DEFLECTION, HIP_DEFLECTION);

    dx -= HIP_L * cos(DEG2RAD * (leg->rotation + leg->hip_angle));
    dy -= HIP_L * sin(DEG2RAD * (leg->rotation + leg->hip_angle));

    // Now dx, dy, dz are the component distances from knee pivot to foot tip
    float leg_foot_ext = sqrt(dx * dx + dy * dy + dz * dz);
    // leg_foot_ext is linear distance from hip pivot to foot tip (long side of iso triangle with ankle_angle as center)
    // law of cosines to the rescue
    float loc_top = LEG_L * LEG_L + FOOT_L * FOOT_L - leg_foot_ext * leg_foot_ext;
    float loc_bottom = 2 * LEG_L * FOOT_L;
    float raw_ankle;
    if (leg_foot_ext > LEG_L + FOOT_L) {
        // full extension
        raw_ankle = 180;
    } else {
        raw_ankle = acos(loc_top / loc_bottom) * RAD2DEG;
    }

    float ankle = 180 - raw_ankle - ANKLE_BIAS;

    float hip_foot_angle = asin(dz / leg_foot_ext) * RAD2DEG;

    float raw_knee = hip_foot_angle - 90 + raw_ankle/2.0;

    // log_debug_vals(leg_foot_ext, ankle, hip_foot_angle, raw_ankle, raw_knee);

    leg->ankle_angle = constrain(ankle, -90, 90);
    leg->knee_angle = constrain(raw_knee, -90, 90);
    
}

void angles_to_xyz(t_leg_pos * leg) {
    leg->x = BODY_PIVOT_R * cos(DEG2RAD * (leg->rotation));
    leg->y = BODY_PIVOT_R * sin(DEG2RAD * (leg->rotation));
    leg->z = HIP_DZ;

    float extension_x = (
        HIP_L + 
        LEG_L * cos(DEG2RAD * (leg->knee_angle)) +
        FOOT_L * cos(DEG2RAD * (leg->knee_angle + (leg->ankle_angle + ANKLE_BIAS))));

    float extension_z = (
        LEG_L * sin(DEG2RAD * (leg->knee_angle)) +
        FOOT_L * sin(DEG2RAD * (leg->knee_angle + (leg->ankle_angle + ANKLE_BIAS))));

    leg->x += extension_x * cos(DEG2RAD * (leg->rotation + leg->hip_angle));
    leg->y += extension_x * sin(DEG2RAD * (leg->rotation + leg->hip_angle));
    leg->z += extension_z;
}

void walk(t_leg_pos * legs[NUM_LEGS], float direction, float speed, float ride_angle) {
    /*
        Use t_leg_pos, angles_to_xyz(), and xyz_to_angles()
        to orchestrate motion one leg at a time. 

        General idea:
        * For any legs in the air, move as far as possible _away_ from bearing "direction".
        * For each leg, compute how far along the floor (x-y) we can move along bearing "direction".
        * Move all legs as far as possible along bearing "direction" without lifting any legs.
        * When a leg reaches its end of motion, lift that leg to a neutral lifted position
        * Return (if still walking, the outer loop will continue the same motion until the next leg needs to move).
    */
    float dx = speed * 10 * cos(direction);
    float dy = speed * 10 * sin(direction);
    Serial.println("dx=" + String(dx) + "; dy=" + String(dy));
    // float avg_z = 0;
    // for (int leg = 0; leg < NUM_LEGS; leg++) {
    //     angles_to_xyz(legs[leg]);
    //     // avg_z += legs[leg]->z;
    //     print_leg(legs[leg]);
    // }
    // // avg_z = avg_z / NUM_LEGS;

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        // detect legs in the air (z < average?)
        if (legs[leg]->knee_angle <= -89) {
            // TODO: Find optimal x,y,z for lifted legs to drop
            legs[leg]->hip_angle = 0;
            legs[leg]->knee_angle = ride_angle;
            legs[leg]->ankle_angle = get_ankle_angle(ride_angle);
            angles_to_xyz(legs[leg]);
            Serial.println("In air: leg " + String(leg));
            print_leg(legs[leg]);
        } else {
            angles_to_xyz(legs[leg]);
            legs[leg]->x += dx;
            legs[leg]->y += dy;
            xyz_to_angles(legs[leg]);
            if (!within_rom(legs[leg])) {
                Serial.println("Exceeded ROM: leg " + String(leg));
                print_leg(legs[leg]);
                // Raise leg to be dropped next step
                legs[leg]->hip_angle = 0;
                legs[leg]->knee_angle = -90;
                legs[leg]->ankle_angle = 90;
            }    
        }
        angles_to_xyz(legs[leg]);
        print_leg(legs[leg]);
    }

    smooth_move_legs(legs, 1000, 10);
    // beeps(1);
    // delay(500);
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
        // stand(leg_ptr, ride_angle);
    }

}
// */
