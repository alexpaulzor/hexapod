
module angles_to_xyz(rotation=0, hip_angle=hip_angle, knee_angle=knee_angle, ankle_angle=ankle_angle) {
	/*
	leg.x = BODY_PIVOT_R * math.cos(DEG2RAD * (leg.rotation))
    leg.y = BODY_PIVOT_R * math.sin(DEG2RAD * (leg.rotation))
    leg.z = HIP_DZ

    extension_x = (
        HIP_L + 
        LEG_L * math.cos(DEG2RAD * (leg.knee_angle)) +
        FOOT_L * math.cos(DEG2RAD * (leg.knee_angle + (leg.ankle_angle + ANKLE_BIAS))))

    leg_z = LEG_L * math.sin(DEG2RAD * (leg.knee_angle))
    foot_z = FOOT_L * math.sin(DEG2RAD * (leg.knee_angle + (leg.ankle_angle + ANKLE_BIAS)))
    extension_z = leg_z + foot_z

    leg.x += extension_x * math.cos(DEG2RAD * (leg.rotation + -leg.hip_angle))
    leg.y += extension_x * math.sin(DEG2RAD * (leg.rotation + -leg.hip_angle))
    leg.z -= extension_z
    */
    leg_x = hexbody_pivot_r * cos(rotation);
    leg_y = hexbody_pivot_r * sin(rotation);
    // leg_z = 0;

    extension_x = (
        hexhip_servo_x + 
        hexleg_l * cos(knee_angle) +
        hexfoot_l * cos(knee_angle + ankle_angle + ankle_bias));

    leg_z = hexleg_l * sin(knee_angle);
    foot_z = hexfoot_l * sin(knee_angle + ankle_angle + ankle_bias);
    extension_z = leg_z + foot_z;

    leg_x2 = leg_x + extension_x * cos(-hip_angle + rotation);
    leg_y2 = leg_y + extension_x * sin(-hip_angle + rotation);
    leg_z2 = -extension_z;

    echo("a_to_xyz:", rotation=rotation,
    	hip_angle=hip_angle, knee_angle=knee_angle, ankle_angle=ankle_angle,
    	leg_x2=leg_x2, leg_y2=leg_y2, leg_z2=leg_z2,
    	extension_x=extension_x);
    # translate([leg_x2, leg_y2, leg_z2]) {
    	sphere(r=2);
    	// % sphere(r=20);
    }
    // % translate([leg_x, leg_y, -leg_z])
    // 	sphere(r=1);
    % xyz_to_angles(rotation, leg_x2, leg_y2, leg_z2);
}
for (rotation=[0:60:hexbody_limit]) {
	angles_to_xyz(rotation);	
}


module xyz_to_angles(rotation=0, x, y, z) {
	dx = x - hexbody_pivot_r * cos(rotation);
    dy = y - hexbody_pivot_r * sin(rotation);
    dz = z;

    hip_angle = atan2(dy, dx) - rotation;

    // echo(rotation=rotation, x=x, y=y, z=z, 
    // 	dx=dx, dy=dy, dz=dz, hip_angle=hip_angle, dx2=dx2, dy2=dy2);
    

    dx2 = dx - hexhip_servo_x * cos(rotation + hip_angle);
    dy2 = dy - hexhip_servo_x * sin(rotation + hip_angle);

    // echo(rotation=rotation, x=x, y=y, z=z, 
    // 	dx=dx, dy=dy, dz=dz, hip_angle=hip_angle, dx2=dx2, dy2=dy2);
    leg_foot_ext = sqrt(dx2 * dx2 + dy2 * dy2 + dz * dz);
    // # leg_foot_ext is linear distance from hip pivot to foot tip (long side of iso triangle with ankle_angle as center)
    // # law of cosines to the rescue
    if (leg_foot_ext > hexleg_l + hexfoot_l)
        echo("cannot reach", leg_foot_ext);
    // # ankle = 180 - math.acos((LEG_L * LEG_L + FOOT_L * FOOT_L - leg_foot_ext * leg_foot_ext) / (2 * LEG_L * FOOT_L)) * RAD2DEG
    ankle = acos(
    	(hexleg_l * hexleg_l + 
    		hexfoot_l * hexfoot_l - 
    		leg_foot_ext * leg_foot_ext) / (
    		2 * hexleg_l * hexleg_l));
    
    // print(f"{locals()}")
    hip_foot_angle = asin(dz / leg_foot_ext);

    /*
	180 = 2 * (hip_foot_angle - knee_angle) + ankle
	(180 - ankle) / 2 = hip_foot_angle - knee_angle
	knee_angle = hip_foot_angle - (180 - ankle)/2
    */
    knee_angle = hip_foot_angle - 90 + ankle/2;
    // knee_angle = (ankle/2 - hip_foot_angle);

    // ankle_angle = min(90, max(-90, ankle + ankle_bias));
    ankle_angle = (ankle - ankle_bias);

    echo("xyz_to_a:", rotation=rotation, 
    	hip_angle=hip_angle, knee_angle=knee_angle, ankle_angle=ankle_angle,
    	x=x, y=y, z=z, 
    	dx=dx, dy=dy, dz=dz, dx2=dx2, dy2=dy2, 
    	ankle=ankle, hip_foot_angle=hip_foot_angle);

    translate([hexbody_pivot_r * cos(rotation), hexbody_pivot_r * sin(rotation), 0]) {
    	rotate([0, 0, rotation + hip_angle]) {
    		cube([hexhip_servo_x, 1, 1]);
    		translate([hexhip_servo_x, 0, 0]) {
    			rotate([0, -knee_angle, 0]) {
    				cube([hexleg_l, 1, 1]);
    				translate([hexleg_l, 0, 0]) {
    					rotate([0, -ankle_angle + ankle_bias, 0]) {
    						# cube([hexfoot_l, 1, 1]);
    					}
    				}

    			}
    		}
    	}
    }
}