#include <Wire.h>
// #include "servobot3.h"

#define BODY_PIVOT_R 188
#define HIP_L 80
#define HIP_DZ 0
#define LEG_L 145
#define FOOT_L 133
#define ANKLE_BIAS 90  // degrees from straight

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
        ", ankle=" + String(leg->ankle_angle));
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
    float raw_ankle;
    if (leg_foot_ext > LEG_L + FOOT_L) {
        // full extension
        raw_ankle = 180;
    } else {
        raw_ankle = acos(loc_top / loc_bottom) * RAD2DEG;
    }

    float ankle = 180 - raw_ankle - ANKLE_BIAS;
    // Serial.println(
    //     "raw_ankle=" + String(raw_ankle) +
    //     "; ankle=" + String(ankle));

    float hip_foot_angle = asin(dz / leg_foot_ext) * RAD2DEG;

    float raw_knee = hip_foot_angle - 90 + raw_ankle/2.0;

    // Serial.println(
    //     "hfa=" + String(hip_foot_angle) +
    //     "; raw_knee=" + String(raw_knee));
    leg->ankle_angle = constrain(ankle, -90, 90);
    leg->knee_angle = constrain(raw_knee, -90, 90);
    
    // if (walk_mode != STEP_MODE_GROUP && leg->rotation == 0) {
        Serial.println("xyz->a");
        print_leg(leg);
    // }
}

void angles_to_xyz(t_leg_pos * leg) {
    // TODO: Fix knee/ankle signs
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
    
    if (walk_mode != STEP_MODE_GROUP && leg->rotation == 0) {
        Serial.println("a->xyz");
        print_leg(leg);
    }
}

void setup() {
    t_leg_pos leg;
    leg.r = 0;
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

