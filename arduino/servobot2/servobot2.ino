#include <Wire.h>
#include "PCA9685.h"
#include "servobot.h"
// #include "kinematics.c"
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
char last_leg = 0;  // last leg to be lifted
char walk_mode = 0;  // 0 = walk as group, 1 = single leg, 2 = articulate leg


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
}

unsigned short get_current_value(int driver, int channel) {
    if (driver == 0)
        return driver0.getChannelPWM(channel);
    else
        return driver1.getChannelPWM(channel);
}

void beep(long duration_ms) {
    // set_motor(
    //     BUZZER_DRIVER, BUZZER_CHANNEL, 
    //     1024);
    digitalWrite(LED_PIN, HIGH);
    analogWrite(BUZZER_PIN, 127);
    delay(duration_ms);
    analogWrite(BUZZER_PIN, 0);
    digitalWrite(LED_PIN, LOW);
    // set_motor(
    //     BUZZER_DRIVER, BUZZER_CHANNEL, 0);
    
}

void beeps(unsigned short count) {
    for (int i = 0; i < count; i++) {
        beep(50);
        delay(50);
    }
}

void print_leg(t_leg_pos * leg) {
    Serial.println(
        "r=" + String(leg->rotation) +
        ", x=" + String(leg->x) +
        ", y=" + String(leg->y) +
        ", z=" + String(leg->z) +
        ", hip=" + String(leg->hip_angle) +
        ", knee=" + String(leg->knee_angle) +
        ", ankle=" + String(leg->ankle_angle));
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
    // Serial.println("Raw hip_angle=" + String(raw_hip) + "; r=" + String(leg->rotation));
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


int within_rom(t_leg_pos * leg) {
    return (
        (-HIP_DEFLECTION < leg->hip_angle && leg->hip_angle < HIP_DEFLECTION) &&
        (-KNEE_DEFLECTION < leg->knee_angle && leg->knee_angle < KNEE_DEFLECTION) &&
        (-ANKLE_DEFLECTION <= leg->ankle_angle && leg->ankle_angle <= ANKLE_DEFLECTION) &&
        abs(leg->ankle_angle + ANKLE_BIAS) > 1); // != 0 because this means the leg is pointed straight away, not articulated
}

int point_within_rom(float rotation, float x, float y, float z) {
    t_leg_pos leg;
    leg.rotation = rotation;
    leg.x = x;
    leg.y = y;
    leg.z = z;
    xyz_to_angles(&leg);
    return within_rom(&leg);
}

void set_angle(unsigned short driver, unsigned short channel, float angle) {
    set_motor(driver, channel, pwmForAngle(angle));
}

void set_all_angles(unsigned short channels[][2], int channels_len, float angle) {
    for (int i = 0; i < channels_len; i++) {
        set_angle(channels[i][0], channels[i][1], angle);
    }
}

void smooth_move_step(
        unsigned short driver, 
        unsigned short channel, 
        float angle, 
        long duration_ms, 
        unsigned long interval_ms) {
    
    unsigned short current_value = get_current_value(driver, channel);
    
    unsigned short target_value = pwmForAngle(angle);
    unsigned short next_value;
    if (duration_ms <= interval_ms)
        next_value = target_value;
    else
        next_value = map(
            interval_ms, 0, duration_ms, 
            current_value, target_value);
    set_motor(driver, channel, next_value);
}

void smooth_move_to(
        unsigned short drivers[], 
        unsigned short channels[], 
        float angles[], 
        int num_channels,
        unsigned long duration_ms) {
    int interval_ms = INTERVAL_MS;
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

void smooth_move_legs(t_leg_pos * legs[NUM_LEGS], long duration_ms) {
    unsigned short drivers[3 * NUM_LEGS];
    unsigned short channels[3 * NUM_LEGS];
    float angles[3 * NUM_LEGS];
    for (int leg = 0; leg < NUM_LEGS; leg++) {   
        drivers[3*leg] = legs[leg]->driver;
        drivers[3*leg + 1] = legs[leg]->driver;
        drivers[3*leg + 2] = legs[leg]->driver;
        channels[3*leg] = legs[leg]->channel;
        channels[3*leg + 1] = legs[leg]->channel + 1;
        channels[3*leg + 2] = legs[leg]->channel + 2;
        angles[3*leg] = legs[leg]->hip_angle;
        angles[3*leg + 1] = legs[leg]->knee_angle;
        angles[3*leg + 2] = legs[leg]->ankle_angle;
    }
    smooth_move_to(
        drivers, channels, angles, 3 * NUM_LEGS,
        duration_ms);
}

void smooth_move_leg(t_leg_pos * leg, long duration_ms) {
    unsigned short drivers[3];
    unsigned short channels[3];
    float angles[3];
       
    drivers[0] = leg->driver;
    drivers[1] = leg->driver;
    drivers[2] = leg->driver;
    channels[0] = leg->channel;
    channels[1] = leg->channel + 1;
    channels[2] = leg->channel + 2;
    angles[0] = leg->hip_angle;
    angles[1] = leg->knee_angle;
    angles[2] = leg->ankle_angle;
    
    smooth_move_to(
        drivers, channels, angles, 3,
        duration_ms);
}

float get_ankle_angle(float knee_angle) {
    if (knee_angle > ANKLE_ACTIVE_ANGLE) 
        return mapf(knee_angle, ANKLE_ACTIVE_ANGLE, MAX_RIDE_ANGLE, KNEE_DEFLECTION, -ANKLE_BIAS);
    return KNEE_DEFLECTION;
}

void stand(t_leg_pos * legs[NUM_LEGS], float ride_angle) {
    /*
        Neutral position at ride_angle (deg).

        ride_angle | z   | knee | ankle | meaning
        ---------- | --- | ---- | ----- | -------
        -90        | -15 | -90  | 90    | sitting on floor, legs up, feet 45 deg down
        -45        | 0   | -45  | 90    | sitting on floor, legs 45 deg up, feet 45 deg out
        0          | 70  | 0    | 90    | legs horizontal, feet 45 deg in
        22.5       | 130 | 22.5 | 67.5  | legs 22.5 deg below horizontal, feet 22.5 deg in from vertical
        45         | 185 | 45   | 0     | legs 45 deg, feet vertical
        90         | 200 | 90   | -45   | legs and feet vertical
    */
    
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        legs[leg]->hip_angle = 0;
        legs[leg]->knee_angle = ride_angle;
        legs[leg]->ankle_angle = get_ankle_angle(ride_angle);
        angles_to_xyz(legs[0]);
    }
    smooth_move_legs(legs, STEP_DURATION);
}

void retract(t_leg_pos * legs[NUM_LEGS]) {
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        legs[leg]->hip_angle = 0;
        legs[leg]->knee_angle = -90;
        legs[leg]->ankle_angle = 90;
        angles_to_xyz(legs[0]);
    }
    smooth_move_legs(legs, STEP_DURATION);
}

void spin(t_leg_pos * legs[NUM_LEGS], float spin_rate, float ride_angle) {
    // TODO: rewrite like walk()
    float hip_angle = -spin_rate * HIP_DEFLECTION;
    float ankle_angle = get_ankle_angle(ride_angle);
    int leg_group = last_leg % 2;
    // TODO: respect walk_mode
    for (int step = 0; step < 6; step++) {
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            if (leg % 2 == leg_group) {
                switch (step) {
                    case 0:  // lift leg and swing opposite turn direction
                        legs[leg]->knee_angle = -90;
                        legs[leg]->hip_angle = -hip_angle;
                        legs[leg]->ankle_angle = 90;
                        break;
                    case 1:  // drop leg
                        legs[leg]->knee_angle = ride_angle;
                        legs[leg]->ankle_angle = ankle_angle;
                        break;
                    case 2:  // turn halfway
                        legs[leg]->hip_angle = 0;
                        break;
                    case 3:  // wait for other legs to drop, finish turn
                        legs[leg]->hip_angle = hip_angle;
                        break;
                    case 4:  // lift leg and center
                        legs[leg]->knee_angle = -90;
                        legs[leg]->hip_angle = 0;
                        legs[leg]->ankle_angle = 90;
                        break;
                    case 5: // drop leg
                        legs[leg]->knee_angle = ride_angle;
                        legs[leg]->ankle_angle = ankle_angle;
                        break;
                    default:
                        break;
                }
            } else {
                switch (step) {
                    case 0:  // turn half
                        legs[leg]->knee_angle = ride_angle;
                        legs[leg]->ankle_angle = ankle_angle;
                        legs[leg]->hip_angle = hip_angle;
                        break;
                    case 1:  // wait for other legs to dop
                        break;
                    case 2:  // lift leg and swing opposite
                        legs[leg]->knee_angle = -90;
                        legs[leg]->ankle_angle = 90;
                        legs[leg]->hip_angle = -hip_angle;
                        break;
                    case 3:  // drop leg
                        legs[leg]->knee_angle = ride_angle;
                        legs[leg]->ankle_angle = ankle_angle;
                        break;
                    case 4: // turn half
                        legs[leg]->hip_angle = 0;
                        break;
                    case 5: // wait for leg to drop
                        break;
                    default:
                        break;
                }
            }
        }
        smooth_move_legs(legs, STEP_DURATION);
        last_leg = (last_leg + 1) % NUM_LEGS;
    }
}

void init_legs(t_leg_pos * legs[NUM_LEGS]) {
    for (unsigned short leg = 0; leg < NUM_LEGS; leg++) {
        legs[leg]->rotation = leg * 60;
        legs[leg]->driver = leg / (NUM_LEGS / 2);
        legs[leg]->channel = (3 * leg) % (3 * NUM_LEGS / 2);
        legs[leg]->hip_angle = 0;
        legs[leg]->knee_angle = -90;
        legs[leg]->ankle_angle = 90;
        angles_to_xyz(legs[leg]);
    }
}

void walk(t_leg_pos * legs[NUM_LEGS], float dx, float dy, float ride_angle) {
    dx *= STEP_SIZE;
    dy *= STEP_SIZE;
    int rom_exceeded = 0;
    int leg = 0;
    int ref_leg = last_leg + 1;
    for (int dleg = 0; dleg < NUM_LEGS; dleg++) {
        leg = (ref_leg + dleg) % NUM_LEGS; 
        if (legs[leg]->knee_angle < MIN_RIDE_ANGLE) {
            legs[leg]->knee_angle = ride_angle;
            legs[leg]->ankle_angle = get_ankle_angle(ride_angle);
            if (abs(legs[leg]->hip_angle) >= abs(HIP_DEFLECTION * HIP_INVERT_RATIO)) {
                legs[leg]->hip_angle = legs[leg]->hip_angle * HIP_INVERT_RATIO;
            } else {
                legs[leg]->hip_angle = 0;
            }
        }
        angles_to_xyz(legs[leg]);
    } 
    smooth_move_legs(legs, STEP_DURATION);
    int all_within_rom = 0;
    while (!all_within_rom && (abs(dx) > MIN_STEP_SIZE || abs(dy) > MIN_STEP_SIZE)) {
        all_within_rom = 1;
        for (int dleg = 0; dleg < NUM_LEGS; dleg++) {
            leg = (ref_leg + dleg) % NUM_LEGS;
            if (within_rom(legs[leg]) 
                    && !point_within_rom(
                        legs[leg]->rotation, 
                        legs[leg]->x + dx, 
                        legs[leg]->y + dy, 
                        legs[leg]->z)) {
                all_within_rom = 0;
                dx = dx / 2;
                dy = dy / 2;
                last_leg = leg;
                break;
            }
        }
    }
    for (int dleg = 0; dleg < NUM_LEGS; dleg++) {
        leg = (last_leg + dleg) % NUM_LEGS;
        legs[leg]->x += dx;
        legs[leg]->y += dy;
        xyz_to_angles(legs[leg]);
        if (rom_exceeded == 0 && !within_rom(legs[leg])) {
            for (int leg2 = leg; leg2 < leg + (walk_mode == STEP_MODE_GROUP ? NUM_LEGS : 1); leg2 += 2) {
                // Raise leg group to be dropped next step
                legs[leg2 % NUM_LEGS]->knee_angle = -90;
                legs[leg2 % NUM_LEGS]->ankle_angle = 90;
                angles_to_xyz(legs[leg2 % NUM_LEGS]);
            }
            last_leg = leg;
            rom_exceeded++;
            // if (walk_mode != STEP_MODE_GROUP) {
            //     beeps(leg+1);
            // }
        }
    }

    smooth_move_legs(legs, STEP_DURATION);
}

void articulate_leg(t_leg_pos * legs[NUM_LEGS], float dhip, float dknee, float dankle) {
    // legs[last_leg]->hip_angle = mapf(lr, -1, 1, HIP_DEFLECTION, -HIP_DEFLECTION);
    // legs[last_leg]->knee_angle = mapf(ride_angle, MIN_RIDE_ANGLE, MAX_RIDE_ANGLE, -KNEE_DEFLECTION, KNEE_DEFLECTION);
    // legs[last_leg]->ankle_angle = mapf(fb, -1, 1, -ANKLE_DEFLECTION, ANKLE_DEFLECTION);
    legs[last_leg]->hip_angle = constrain(
        legs[last_leg]->hip_angle + mapf(dhip, -1, 1, -HIP_DEFLECTION/2, HIP_DEFLECTION/2),
        -HIP_DEFLECTION, HIP_DEFLECTION);
    legs[last_leg]->knee_angle = constrain(
        legs[last_leg]->knee_angle + mapf(dknee, -1, 1, -KNEE_DEFLECTION/2, KNEE_DEFLECTION/2),
        -KNEE_DEFLECTION, KNEE_DEFLECTION);
    legs[last_leg]->ankle_angle = constrain(
        legs[last_leg]->ankle_angle + mapf(dankle, -1, 1, -ANKLE_DEFLECTION/2, ANKLE_DEFLECTION/2),
        -ANKLE_DEFLECTION, ANKLE_DEFLECTION);
    // angles_to_xyz(legs[last_leg]);
    // legs[last_leg]->y += mapf(lr, -1, 1, -STEP_SIZE/2, STEP_SIZE/2);
    // legs[last_leg]->z += mapf(ride_angle, MIN_RIDE_ANGLE, MAX_RIDE_ANGLE, -STEP_SIZE/2, STEP_SIZE/2);
    // legs[last_leg]->x += mapf(fb, -1, 1, -STEP_SIZE/2, STEP_SIZE/2);
    // xyz_to_angles(legs[last_leg]);

    smooth_move_leg(legs[last_leg], STEP_DURATION);
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
    pinMode(PCA_ENABLE_PIN, OUTPUT);
    
    Serial.begin(BAUD_RATE);

    for (int leg = 0; leg < NUM_LEGS; leg++)
        leg_ptr[leg] = &legs[leg];
    init_legs(leg_ptr);
    digitalWrite(PCA_ENABLE_PIN, LOW);
    delay(500);
    retract(leg_ptr);
    beeps(3);
    delay(500);
    // digitalWrite(PCA_ENABLE_PIN, HIGH);
}

void loop() {
    // unsigned long loop_start = millis();
    int channel_values[NUM_CHANNELS];
    // Print latest valid values from all channels
    for (int channel = 0; channel < NUM_CHANNELS; channel++) {
        int value = constrain(
            ppm.latestValidChannelValue(channel + 1, 0),
            PPM_LOW, PPM_HIGH);
        // Serial.print(String(value) + " ");
        if (value >= PPM_LOW && value <= PPM_HIGH) {
            // if (value != channel_values[channel]) {
            //     last_ppm_signal = loop_start;
            // }
            channel_values[channel] = value;
        }
    }
    

    walk_mode = constrain(
        map(channel_values[CHANNEL_STEP_MODE], PPM_LOW, PPM_HIGH, 0, 3),
        0, 2);
    Serial.println("walk_mode: " + String(walk_mode));
    float ride_angle = mapf(
        channel_values[CHANNEL_RIDE_HEIGHT],
        PPM_LOW, PPM_HIGH, MIN_RIDE_ANGLE, MAX_RIDE_ANGLE);
    float spin_rate = mapf(
            channel_values[CHANNEL_TURN], 
            PPM_LOW, PPM_HIGH, 
            -1.0, 1.0);
    float fb = mapf(channel_values[CHANNEL_FB], PPM_LOW, PPM_HIGH, -1.0, 1.0);
    float lr = mapf(channel_values[CHANNEL_LR], PPM_LOW, PPM_HIGH, -1.0, 1.0);
    // TODO: command loss timeout

    if (channel_values[CHANNEL_ENABLE_DRIVE] < PPM_CENTER) {
        // Drive disabled
        // digitalWrite(PCA_ENABLE_PIN, HIGH);
        delay(250);
        return;
    }
    // digitalWrite(PCA_ENABLE_PIN, LOW);
    if (walk_mode == STEP_MODE_INDIV) {
        last_leg = constrain(
            map(channel_values[CHANNEL_LEG_SELECT], PPM_LOW, PPM_HIGH, 0, 6), 
            0, 5);
        articulate_leg(leg_ptr, spin_rate, lr, fb);
    } else if (abs(fb) > DEADZONE_RATIO || abs(lr) > DEADZONE_RATIO) {
        walk(leg_ptr, lr, fb, ride_angle);
    } else if (abs(spin_rate) > DEADZONE_RATIO) {
        // Spin in place
        spin(leg_ptr, spin_rate, ride_angle);
    } else {
        stand(leg_ptr, ride_angle);
    }

}
// */
