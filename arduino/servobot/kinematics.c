#include "servobot.h"

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

void xyz_to_angles(t_leg_pos * leg) {
	float dx = leg->x - BODY_PIVOT_R * cos(DEG2RAD * (leg->rotation));
	float dy = leg->y - BODY_PIVOT_R * sin(DEG2RAD * (leg->rotation));
	float dz = leg->z - HIP_DZ;

	leg->hip_angle = constrain(
        (atan(dy / dx) * RAD2DEG - leg->rotation),
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

int within_rom(t_leg_pos * leg) {
    return (
        (-HIP_DEFLECTION < leg->hip_angle && leg->hip_angle < HIP_DEFLECTION) &&
        (-KNEE_DEFLECTION < leg->knee_angle && leg->knee_angle < KNEE_DEFLECTION) &&
        (-ANKLE_DEFLECTION <= leg->ankle_angle && leg->ankle_angle <= ANKLE_DEFLECTION) &&
        leg->ankle_angle != 0); // != 0 because this means the leg is pointed straight away, not articulated
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
        unsigned short drivers[], 
        unsigned short channels[], 
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

    unsigned short drivers[3 * NUM_LEGS];
    unsigned short channels[3 * NUM_LEGS];
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

void smooth_move_legs(t_leg_pos * legs[NUM_LEGS], long duration_ms, unsigned long interval_ms) {
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
        duration_ms, interval_ms);
}

void smooth_move_leg(t_leg_pos * leg, long duration_ms, unsigned long interval_ms) {
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
        duration_ms, interval_ms);
}

void smooth_move_sequence(float leg_angles[][NUM_LEGS][3], int num_moves, long duration_ms, unsigned long interval_ms) {
    for (int i = 0; i < num_moves; i++) {
        smooth_move_all(leg_angles[i], duration_ms, interval_ms);
    }
}
/*
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
}*/

float get_ankle_angle(float knee_angle) {
    if (knee_angle > 0) 
        return map(knee_angle, 0, 90, 90, -45);
    return 90;
}

float get_step_duration_ms(float speed) {
    return map(abs(speed), 0, 1, 1000, 200);
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
    smooth_move_legs(legs, get_step_duration_ms(1.0), 5);
}

void retract(t_leg_pos * legs[NUM_LEGS]) {
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        legs[leg]->hip_angle = 0;
        legs[leg]->knee_angle = -90;
        legs[leg]->ankle_angle = 90;
        angles_to_xyz(legs[0]);
    }
    smooth_move_legs(legs, get_step_duration_ms(1.0), 5);
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
/*
void walk_derpy(float direction, float speed, float ride_angle) {

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
}*/

void init_legs(t_leg_pos * legs[NUM_LEGS]) {
    for (unsigned short leg = 0; leg < NUM_LEGS; leg++) {
        legs[leg]->rotation = leg * 60;
        legs[leg]->driver = leg / (NUM_LEGS / 2);
        legs[leg]->channel = (3 * leg) % (3 * NUM_LEGS / 2);
        legs[leg]->hip_angle = 0;
        // get_current_value(
        //     legs[leg]->driver, legs[leg]->channel);
        legs[leg]->knee_angle = -90;
        // get_current_value(
        //     legs[leg]->driver, legs[leg]->channel + 1);
        legs[leg]->ankle_angle = 90;
        // get_current_value(
        //     legs[leg]->driver, legs[leg]->channel + 2);
        angles_to_xyz(legs[leg]);
    }
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
    // float avg_z = 0;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        angles_to_xyz(legs[leg]);
        // avg_z += legs[leg]->z;
    }
    // avg_z = avg_z / NUM_LEGS;

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        // detect legs in the air (z < average?)
        if (!within_rom(legs[leg])) {
            // TODO: Find optimal x,y,z for lifted legs to drop
            legs[leg]->hip_angle = 0;
            legs[leg]->knee_angle = ride_angle;
            legs[leg]->ankle_angle = get_ankle_angle(ride_angle);
        }
    }
    

    for (int leg = 0; leg < 1+0*NUM_LEGS; leg++) {
        legs[leg]->x += dx;
        legs[leg]->y += dy;
        xyz_to_angles(legs[leg]);
        if (!within_rom(legs[leg])) {
            // Raise leg to be dropped next step
            legs[leg]->hip_angle = 0;
            legs[leg]->knee_angle = -90;
            legs[leg]->ankle_angle = -90;
            angles_to_xyz(legs[leg]);
        }
    }

    smooth_move_legs(legs, 2000, 5);
}
