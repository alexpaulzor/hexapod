#include "servobot.h"

/*
def angles_to_xyz(leg):
    leg.x = BODY_PIVOT_R * math.cos(DEG2RAD * (leg.rotation))
    leg.y = BODY_PIVOT_R * math.sin(DEG2RAD * (leg.rotation))
    leg.z = HIP_DZ

    extension_x = (
        HIP_L + 
        LEG_L * math.cos(DEG2RAD * (leg.knee_angle)) +
        FOOT_L * math.cos(DEG2RAD * (leg.knee_angle + (leg.ankle_angle + ANKLE_BIAS))))

    extension_z = (
        LEG_L * math.sin(DEG2RAD * (leg.knee_angle)) +
        FOOT_L * math.sin(DEG2RAD * (leg.knee_angle + (leg.ankle_angle + ANKLE_BIAS))))

    leg.x += extension_x * math.cos(DEG2RAD * (leg.rotation + leg.hip_angle))
    leg.y += extension_x * math.sin(DEG2RAD * (leg.rotation + leg.hip_angle))
    leg.z += extension_z
    print(f"{locals()}")
    return leg
*/

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

/*
def xyz_to_angles(leg):
    """Prefer vertical feet (ankle_angle + ANKLE_BIAS ~= -knee_angle)
    """

    dx = leg.x - BODY_PIVOT_R * math.cos(DEG2RAD * (leg.rotation));
    dy = leg.y - BODY_PIVOT_R * math.sin(DEG2RAD * (leg.rotation));
    dz = leg.z - HIP_DZ;

    leg.hip_angle = (math.atan2(dy, dx) * RAD2DEG - leg.rotation) % 180;

    dx -= HIP_L * math.cos(DEG2RAD * (leg.rotation + leg.hip_angle))
    dy -= HIP_L * math.sin(DEG2RAD * (leg.rotation + leg.hip_angle))

    leg_foot_ext = math.sqrt(dx * dx + dy * dy + dz * dz);
    # leg_foot_ext is linear distance from hip pivot to foot tip (long side of iso triangle with ankle_angle as center)
    # law of cosines to the rescue
    leg.ankle_angle = 180 - math.acos((LEG_L * LEG_L + FOOT_L * FOOT_L - leg_foot_ext * leg_foot_ext) / (2 * LEG_L * FOOT_L)) * RAD2DEG - ANKLE_BIAS

    leg.knee_angle = math.asin(dz / leg_foot_ext) * RAD2DEG
    print(f"{locals()}")
    
    return leg
*/

void xyz_to_angles(t_leg_pos * leg) {
	float dx = leg->x - BODY_PIVOT_R * cos(DEG2RAD * (leg->rotation));
	float dy = leg->y - BODY_PIVOT_R * sin(DEG2RAD * (leg->rotation));
	float dz = leg->z - HIP_DZ;

    // TODO: make sure atan sign is right
	leg->hip_angle = (atan(dy / dx) * RAD2DEG - leg->rotation);
	float extension_x = sqrt(dx * dx + dy * dy);

	dx -= HIP_L * cos(DEG2RAD * (leg->rotation + leg->hip_angle));
    dy -= HIP_L * sin(DEG2RAD * (leg->rotation + leg->hip_angle));

    float leg_foot_ext = sqrt(dx * dx + dy * dy + dz * dz);
    // leg_foot_ext is linear distance from hip pivot to foot tip (long side of iso triangle with ankle_angle as center)
    // law of cosines to the rescue
    leg->ankle_angle = 180 - acos(
        (LEG_L * LEG_L + FOOT_L * FOOT_L - leg_foot_ext * leg_foot_ext) / 
        (2 * LEG_L * FOOT_L)) * RAD2DEG - ANKLE_BIAS;

    leg->knee_angle = asin(dz / leg_foot_ext) * RAD2DEG;
}


void set_angle(int driver, int channel, float angle) {
    set_motor(driver, channel, pwmForAngle(angle));
}

void set_all_angles(int channels[][2], int channels_len, float angle) {
    for (int i = 0; i < channels_len; i++) {
        set_angle(channels[i][0], channels[i][1], angle);
    }
}

void smooth_move_step(
        int driver, 
        int channel, 
        float angle, 
        long duration_ms, 
        unsigned long interval_ms) {
    
    unsigned short current_value = get_current_value(driver, channel);  // = DRIVERS[driver].getChannelPWM(channel);
    // if (driver == 0)
    //     current_value = driver0.getChannelPWM(channel);
    // else
    //     current_value = driver1.getChannelPWM(channel);
    
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

void smooth_move_all(
        float leg_angles[NUM_LEGS][3], long duration_ms, unsigned long interval_ms) {
    // leg_angles = {{hip0, knee0, ankle0}, ...}

    int drivers[3 * NUM_LEGS];
    int channels[3 * NUM_LEGS];
    float angles[3 * NUM_LEGS];
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        int driver = leg / (NUM_LEGS / 2);
        for (int motor = 0; motor < 3; motor++) {
            drivers[3 * leg + motor] = driver;
            channels[3 * leg + motor] = (3 * leg + motor) % (3 * NUM_LEGS / 2);
            angles[3 * leg + motor] = leg_angles[leg][motor];
        } 
    }
    smooth_move_to(
        drivers, channels, angles, 3 * NUM_LEGS,
        duration_ms, interval_ms);
}

void smooth_move_sequence(float leg_angles[][NUM_LEGS][3], int num_moves, long duration_ms, unsigned long interval_ms) {
    for (int i = 0; i < num_moves; i++) {
        smooth_move_all(leg_angles[i], duration_ms, interval_ms);
    }
}

void flatten() {
    // set_all_angles(DRV_CH_HIPS, NUM_LEGS, 0);
    // set_all_angles(DRV_CH_KNEES, NUM_LEGS, 0);
    // set_all_angles(DRV_CH_ANKLES, NUM_LEGS, 0);
    float flat_angles[NUM_LEGS][3] = {
        {0, 0, -45}, 
        {0, 0, -45}, 
        {0, 0, -45}, 
        {0, 0, -45}, 
        {0, 0, -45}, 
        {0, 0, -45}
    };
    smooth_move_all(flat_angles, 100, 5);
}

float get_ankle_angle(float knee_angle) {
    if (knee_angle > 0) 
        return map(knee_angle, 0, 90, 90, -45);
    return 90;
}

float get_step_duration_ms(float speed) {
    return map(abs(speed), 0, 1, 3000, 200);
}

void stand(float ride_angle) {
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
    
    float ankle_angle = get_ankle_angle(ride_angle);
    float stand_angles[NUM_LEGS][3] = {
        {0, ride_angle, ankle_angle}, 
        {0, ride_angle, ankle_angle}, 
        {0, ride_angle, ankle_angle}, 
        {0, ride_angle, ankle_angle}, 
        {0, ride_angle, ankle_angle}, 
        {0, ride_angle, ankle_angle}
    };
    smooth_move_all(stand_angles, get_step_duration_ms(1.0), 5);
}

void spin(float spin_rate, float ride_angle) {
    // spin_rate [-1, 1] (-1 = CCW, 1 = CW)
    float hip_angle = -abs(spin_rate)/spin_rate * HIP_DEFLECTION;

    float ankle_angle = get_ankle_angle(ride_angle);

    float leg_angles[][NUM_LEGS][3] = {
        // {  // stand neutral
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}},
        {  // lift leg group 0
            {0, -90, ankle_angle}, 
            {0, ride_angle, ankle_angle}, 
            {0, -90, ankle_angle}, 
            {0, ride_angle, ankle_angle}, 
            {0, -90, ankle_angle}, 
            {0, ride_angle, ankle_angle}},
        {  // move unlifted group 1 forward, lifted leg group 0 backward
            {-hip_angle, -90, ankle_angle}, 
            {hip_angle, ride_angle, ankle_angle}, 
            {-hip_angle, -90, ankle_angle}, 
            {hip_angle, ride_angle, ankle_angle}, 
            {-hip_angle, -90, ankle_angle}, 
            {hip_angle, ride_angle, ankle_angle}},
        {  // unlift leg group 0
            {-hip_angle, ride_angle, ankle_angle}, 
            {hip_angle, ride_angle, ankle_angle}, 
            {-hip_angle, ride_angle, ankle_angle}, 
            {hip_angle, ride_angle, ankle_angle}, 
            {-hip_angle, ride_angle, ankle_angle}, 
            {hip_angle, ride_angle, ankle_angle}},
        {  // lift leg group 1
            {-hip_angle, ride_angle, ankle_angle}, 
            {hip_angle, -90, ankle_angle}, 
            {-hip_angle, ride_angle, ankle_angle}, 
            {hip_angle, -90, ankle_angle}, 
            {-hip_angle, ride_angle, ankle_angle}, 
            {hip_angle, -90, ankle_angle}},
        {  // move unlifted group 0 forward, lifted leg group 1 backward
            {hip_angle, ride_angle, ankle_angle}, 
            {-hip_angle, -90, ankle_angle}, 
            {hip_angle, ride_angle, ankle_angle}, 
            {-hip_angle, -90, ankle_angle}, 
            {hip_angle, ride_angle, ankle_angle},
            {-hip_angle, -90, ankle_angle}}, 
        {  // drop leg group 1
            {hip_angle, ride_angle, ankle_angle}, 
            {-hip_angle, ride_angle, ankle_angle}, 
            {hip_angle, ride_angle, ankle_angle}, 
            {-hip_angle, ride_angle, ankle_angle}, 
            {hip_angle, ride_angle, ankle_angle},
            {-hip_angle, ride_angle, ankle_angle}}, 
        {  // lift leg group 0
            {hip_angle, -90, ankle_angle}, 
            {-hip_angle, ride_angle, ankle_angle}, 
            {hip_angle, -90, ankle_angle}, 
            {-hip_angle, ride_angle, ankle_angle}, 
            {hip_angle, -90, ankle_angle},
            {-hip_angle, ride_angle, ankle_angle}}, 
        // {  // center again
        //     {0, -90, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, -90, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, -90, ankle_angle},
        //     {0, ride_angle, ankle_angle}}, 
        // {  // stand neutral
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}},
        // {  // move unlifted group 0 forward, lifted leg group 1 backward
        //     {hip_angle, -ride_angle, ankle_angle}, 
        //     {-hip_angle, ride_angle, ankle_angle}, 
        //     {hip_angle, -ride_angle, ankle_angle}, 
        //     {-hip_angle, ride_angle, ankle_angle}, 
        //     {hip_angle, -ride_angle, ankle_angle}, 
        //     {-hip_angle, ride_angle, ankle_angle}},
    };
    smooth_move_sequence(
        leg_angles, 
        sizeof(leg_angles)/sizeof(leg_angles[0]),
        get_step_duration_ms(spin_rate), 5);
}

void walk(float direction, float speed, float ride_angle) {

    float ankle_angle = get_ankle_angle(ride_angle);

    // TODO: adjust "leading" leg based on direction

    // This assumes walking towards leg 0
    float leg_angles[][NUM_LEGS][3] = {
        // {  // stand neutral
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}, 
        //     {0, ride_angle, ankle_angle}},
        {  // lift leg group 0
            {0, -90, ankle_angle}, 
            {0, ride_angle, ankle_angle}, 
            {0, -90, ankle_angle}, 
            {0, ride_angle, ankle_angle}, 
            {0, -90, ankle_angle}, 
            {0, ride_angle, ankle_angle}},
        {  // move group 1 "forward", group 0 "backward"
            {HIP_DEFLECTION, -90, ankle_angle}, 
            {-HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {0, -90, ankle_angle - HIP_DEFLECTION}, 
            {HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {-HIP_DEFLECTION, -90, ankle_angle}, 
            {0, ride_angle + HIP_DEFLECTION/2, ankle_angle - HIP_DEFLECTION}},
        {  // bring down group 0
            {HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {-HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {0, ride_angle + HIP_DEFLECTION/2, ankle_angle - HIP_DEFLECTION}, 
            {HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {-HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {0, ride_angle + HIP_DEFLECTION/2, ankle_angle - HIP_DEFLECTION}},
        {  // lift group 1
            {HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {-HIP_DEFLECTION, -90, ankle_angle}, 
            {0, ride_angle + HIP_DEFLECTION/2, ankle_angle - HIP_DEFLECTION}, 
            {HIP_DEFLECTION, -90, ankle_angle}, 
            {-HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {0, -90, ankle_angle - HIP_DEFLECTION}},
        {  // move group 0 "forward", group 1 "backward"
            {-HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {HIP_DEFLECTION, -90, ankle_angle}, 
            {0, ride_angle, ankle_angle}, 
            {-HIP_DEFLECTION, -90, ankle_angle}, 
            {HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {0, -90, ankle_angle}},
        {  // lower group 1
            {-HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {0, ride_angle, ankle_angle}, 
            {-HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {0, ride_angle, ankle_angle}},
        {  // lift group 0
            {-HIP_DEFLECTION, -90, ankle_angle}, 
            {HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {0, -90, ankle_angle}, 
            {-HIP_DEFLECTION, ride_angle, ankle_angle}, 
            {HIP_DEFLECTION, -90, ankle_angle}, 
            {0, ride_angle + HIP_DEFLECTION/2, ankle_angle - HIP_DEFLECTION}},
        

    };

    smooth_move_sequence(
        leg_angles, 
        sizeof(leg_angles)/sizeof(leg_angles[0]),
        10*get_step_duration_ms(speed), 5);
    // delay(1000);
}

void plan_steps() {
    for (int leg_group = 0; leg_group < 2; leg_group++) {

    }
}

/*
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
        smooth_move_all(dance_moves[i], 1500, 5);
    }
}
// */