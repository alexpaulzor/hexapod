#include "trig.h"
// #include <Wire.h>
// #include "PCA9685.h"

#ifndef SERVOBOT_H
#define SERVOBOT_H

// Physical parameters
#define BODY_PIVOT_R 100
#define HIP_L 38
#define HIP_DZ 0
#define LEG_L 100
#define FOOT_L 100
#define ANKLE_BIAS 45

// Software limits
#define HIP_DEFLECTION 30
#define KNEE_DEFLECTION 90
#define ANKLE_DEFLECTION 90

// Servo configuration
#define MIN_PWM 102
#define MAX_PWM 470

#define MAX_DEG_PER_S (60 / 0.15)
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


// R/C stuff
#define PPM_LOW 1000
#define PPM_HIGH 2000
#define PPM_CENTER ((PPM_LOW + PPM_HIGH) / 2)
#define DEADZONE_RATIO 0.05
#define INTERRUPT_PIN 3
#define PPM_TIMEOUT_MS 1000

#define BUZZER_PIN 9

#define NUM_CHANNELS 7

#define CHANNEL_R_EW 0
#define CHANNEL_R_NS 1
#define CHANNEL_L_NS 2
#define CHANNEL_L_EW 3
#define CHANNEL_SWA 4
#define CHANNEL_SWB 5
#define CHANNEL_DIAL_VRA 6

#define CHANNEL_RIDE_HEIGHT CHANNEL_L_NS
#define CHANNEL_TURN CHANNEL_L_EW
#define CHANNEL_FB CHANNEL_R_NS
#define CHANNEL_LR CHANNEL_R_EW
// #define CHANNEL_AC_RELAY CHANNEL_SWB
#define CHANNEL_ENABLE_DRIVE CHANNEL_SWA
// #define CHANNEL_HEIGHT CHANNEL_DIAL_VRA

// Mathematical constants
#define PI 3.141592
const float DEG2RAD = PI / 180.0f;
const float RAD2DEG = 180.0f / PI;

// Data types
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

// Function definitions
void angles_to_xyz(t_leg_pos * leg);
void xyz_to_angles(t_leg_pos * leg);

float get_step_duration_ms(float speed);
unsigned short pwmForAngle(float angle);
void set_motor(int driver, int channel, unsigned short pwm);
unsigned short get_current_value(int driver, int channel);

// void log_debug_vals(float leg_foot_ext, float ankle, float hip_foot_angle, float raw_ankle, float raw_knee);

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void beep(long duration_ms) {
	analogWrite(BUZZER_PIN, 127);
	delay(duration_ms);
	analogWrite(BUZZER_PIN, 0);
}

void beeps(unsigned short count) {
	for (int i = 0; i < count; i++) {
		beep(50);
		delay(50);
	}
}

#endif // SERVOBOT_H