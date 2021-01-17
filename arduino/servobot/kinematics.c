#include "trig.h"

#define BODY_PIVOT_R 100
#define HIP_L 38
#define HIP_DZ 0
#define LEG_L 100
#define FOOT_L 100
#define MAX_DEG_PER_S (60 / 0.15)

void angles_to_xyz(t_leg_pos * leg) {
	leg->x = BODY_PIVOT_R * cos(DEG2RAD * (leg->rotation));
	leg->y = BODY_PIVOT_R * sin(DEG2RAD * (leg->rotation));
	leg->z = HIP_DZ;

	float extension_x = (
		HIP_L + 
		LEG_L * cos(DEG2RAD * (leg->knee_angle)) +
		FOOT_L * cos(DEG2RAD * (leg->knee_angle + leg->ankle_angle)));

	float extension_z = (
		LEG_L * sin(DEG2RAD * (leg->knee_angle)) +
		FOOT_L * sin(DEG2RAD * (leg->knee_angle + leg->ankle_angle)));

	leg->x += extension_x * cos(DEG2RAD * (leg->rotation + leg->hip_angle));
	leg->y += extension_x * sin(DEG2RAD * (leg->rotation + leg->hip_angle));
	leg->z += extension_z;
}

void xyz_to_angles(t_leg_pos * leg) {

	float dx = leg->x - BODY_PIVOT_R * cos(DEG2RAD * (leg->rotation));
	float dy = leg->y - BODY_PIVOT_R * sin(DEG2RAD * (leg->rotation));
	float dz = leg->z - HIP_DZ;

	leg->hip_angle = atan(dy / dx) * RAD2DEG - leg->rotation;
	float extension_x = sqrt(dx * dx + dy * dy);

	// TODO: system of equations this shiite.
}