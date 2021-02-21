#include <Wire.h>
// #include "servobot3.h"

#define PI 3.141592
const float DEG2RAD = PI / 180.0f;
const float RAD2DEG = 180.0f / PI;

#define BODY_PIVOT_R 188
#define HIP_L 80
#define HIP_DZ 0
#define LEG_L 145
#define FOOT_L 133
#define ANKLE_BIAS (28.8 - 90)  // degrees from straight, 2 teeth on 25T shaft

// Software limits
#define HIP_DEFLECTION 45
#define KNEE_DEFLECTION 90
#define ANKLE_DEFLECTION 90
#define BAUD_RATE 115200

typedef struct {
    // Angles are in degrees
    // TODO: use native radians
    float rotation;
    float hip_angle;
    float knee_angle;
    float ankle_angle;
    float x;
    float y;
    float z;
    // TODO: use pointer to driver object
    unsigned short driver;
    unsigned short channel;
} t_leg_pos;

void print_leg(t_leg_pos * leg) {
    Serial.println(
        "r=" + String(leg->rotation) +
        ", x=" + String(leg->x) +
        ", y=" + String(leg->y) +
        ", z=" + String(leg->z) +
        ", hip=" + String(leg->hip_angle) +
        ", knee=" + String(leg->knee_angle) +
        ", ankle=" + String(leg->ankle_angle) +
        ", a'=" + String(leg->ankle_angle + ANKLE_BIAS + 180));
}

void xyz_to_angles(t_leg_pos * leg) {
    // TODO: Fix knee/ankle signs
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
    // leg_foot_ext is linear distance from hip pivot to foot tip (side of triangle opposite ankle_angle)
    // law of cosines to the rescue

    float loc_top = 1.0 * (LEG_L * LEG_L) + (FOOT_L * FOOT_L) - (leg_foot_ext * leg_foot_ext);
    float loc_bottom = 2.0 * LEG_L * FOOT_L;
    // Serial.println(
    //     "dx=" + String(dx) +
    //     "; dy=" + String(dy) +
    //     "; dz=" + String(dz) +
    //     "; lfe=" + String(leg_foot_ext) +
    //     "; loc_top=" + String(loc_top) +
    //     "; loc_bottom=" + String(loc_bottom));
    float ankle_prime;
    if (leg_foot_ext > LEG_L + FOOT_L) {
        // full extension
        ankle_prime = 180;
    } else {
        ankle_prime = acos(loc_top / loc_bottom) * RAD2DEG;
    }

    if (ankle_prime < ANKLE_BIAS || ankle_prime > 180) {
        Serial.println("ankle_prime raw = " + String(ankle_prime));
        print_leg(leg);
    }
    // ankle_prime = constrain(ankle_prime, ANKLE_BIAS, 180);

    float hip_foot_angle = asin(dz / leg_foot_ext) * RAD2DEG;

    float knee_prime = asin(FOOT_L * sin(ankle_prime * DEG2RAD) / leg_foot_ext) * RAD2DEG;

    // a->xyz: r=0.00, x=536.66, y=0.00, z=64.80, hip=0.00, knee=4.08, ankle=-64.08
    // ankle_prime=134.89; ankle=16.31; hfa=-14.00; raw_knee=-36.56
    // xyz->a: r=0.00, x=527.86, y=0.00, z=64.80, hip=0.00, knee=-36.56, ankle=16.31

    float ankle = ankle_prime - 180 - ANKLE_BIAS;
    // Serial.println(
    //     "ankle_prime=" + String(ankle_prime) +
    //     "; ankle=" + String(ankle));

    
    // Serial.println(
    //     "hfa=" + String(hip_foot_angle) +
    //     "; raw_knee=" + String(raw_knee));
    leg->ankle_angle = constrain(ankle, -90, 90);
    leg->knee_angle = constrain(hip_foot_angle + knee_prime, -90, 90);

    float toe_angle = asin(LEG_L * sin(ankle_prime * DEG2RAD) / leg_foot_ext) * RAD2DEG;
    
    // if (walk_mode != STEP_MODE_GROUP && leg->rotation == 0) {
       // if (leg->rotation == 0) {
        Serial.println(
            "xyz->a: ankle_prime=" + String(ankle_prime) +
            "; ankle=" + String(ankle) +
            "; hfa=" + String(hip_foot_angle) +
            "; knee_prime=" + String(knee_prime) + 
            "; lfe=" + String(leg_foot_ext) +
            "; loc_top=" + String(loc_top) +
            "; loc_bottom=" + String(loc_bottom) +
            "; dx=" + String(dx) +
            "; dy=" + String(dy) +
            "; dz=" + String(dz) +
            "; toe=" + String(toe_angle) +
            "; ap+kp+t=" + String(toe_angle + ankle_prime + knee_prime));
            
        Serial.print("xyz->a: ");
        print_leg(leg);
    // }
}

void angles_to_xyz(t_leg_pos * leg) {
    // TODO: Fix knee/ankle signs
    leg->x = BODY_PIVOT_R * cos(DEG2RAD * (leg->rotation));
    leg->y = BODY_PIVOT_R * sin(DEG2RAD * (leg->rotation));
    leg->z = HIP_DZ;

    float extension_horiz = (
        HIP_L + 
        LEG_L * cos(DEG2RAD * (leg->knee_angle)) +
        FOOT_L * cos(DEG2RAD * (leg->knee_angle + (leg->ankle_angle + ANKLE_BIAS))));

    float extension_vert = (
        LEG_L * sin(DEG2RAD * (leg->knee_angle)) +
        FOOT_L * sin(DEG2RAD * (leg->knee_angle + (leg->ankle_angle + ANKLE_BIAS))));

    float dx = extension_horiz * cos(DEG2RAD * (leg->rotation + leg->hip_angle));
    float dy = extension_horiz * sin(DEG2RAD * (leg->rotation + leg->hip_angle));
    leg->x += dx;
    leg->y += dy;
    leg->z += extension_vert;
    
    // if (walk_mode != STEP_MODE_GROUP && leg->rotation == 0) {
    // if (leg->rotation == 0) {
        Serial.println(
            "a->xyz: extension_horiz=" + String(extension_horiz) +
            "; extension_vert=" + String(extension_vert) +
            "; dx=" + String(dx) +
            "; dy=" + String(dy));
        Serial.print("a->xyz: ");
        print_leg(leg);
    // }
}

void setup() {
    Serial.begin(BAUD_RATE);
    t_leg_pos leg;
    leg.rotation = 0;
    leg.hip_angle = 0;
    leg.knee_angle = 90;
    leg.ankle_angle = -90;
    angles_to_xyz(&leg);
    xyz_to_angles(&leg);
    angles_to_xyz(&leg);
    xyz_to_angles(&leg);
}

void loop() {

}

