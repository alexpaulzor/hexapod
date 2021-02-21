// include <servobot3.scad>;

BODY_PIVOT_R = 188;
HIP_L = 80;
LEG_L = 522; // 145;
FOOT_L = 133;

ankle_bias = 28.8; // 90 - 24; // from perpendicular


// r=240.00;
// x=-327.99;
// y=-259.55;
// z=-84.91;
// hip=-37.54;
// knee=-72.23;
// ankle=68.24;

r=0.00;
// x=517.65;
// y=88.88;
// z=-69.54;
hip=20;
knee=-50;
ankle=20;

function angles_to_x(r, h, k, a) = 
    BODY_PIVOT_R * cos(r) +
    HIP_L * cos(r + h) +
    LEG_L * cos(k) * cos(r + h) +
    FOOT_L * cos(k + a + ankle_bias) * cos(r + h);

function angles_to_y(r, h, k, a) = 
    BODY_PIVOT_R * sin(r) +
    HIP_L * sin(r + h) +
    LEG_L * cos(k) * sin(r + h) +
    FOOT_L * cos(k + a + ankle_bias) * sin(r + h);

function angles_to_z(r, h, k, a) = 
    -LEG_L * sin(k) +
    -FOOT_L * sin(k + a + ankle_bias);

function angles_to_p(r, h, k, a) = [
    angles_to_x(r, h, k, a),
    angles_to_y(r, h, k, a),
    angles_to_z(r, h, k, a)];

module angles_to_xyz(r=r, hip=hip, knee=knee, ankle=ankle) {
    rotate([0, 0, r]) {
        cube([BODY_PIVOT_R, 5, 1]);
        translate([BODY_PIVOT_R, 0, 0]) {
            rotate([0, 0, hip]) {
                cube([HIP_L, 5, 1]);
                translate([HIP_L, 0, 0]) {
                    rotate([0, knee, 0]) {
                        cube([LEG_L, 5, 1]);
                        translate([LEG_L, 0, 0]) {
                            rotate([0, ankle + ankle_bias, 0]) {
                                cube([FOOT_L, 5, 1]);
                            }
                        }
                    }
                }
            }
        }
    }
    echo(
        "angles_to_xyz: ",
        r=r, hip=hip, knee=knee, ankle=ankle,
        x=angles_to_x(r, hip, knee, ankle),
        y=angles_to_y(r, hip, knee, ankle),
        z=angles_to_z(r, hip, knee, ankle));
    # translate(angles_to_p(r, hip, knee, ankle)) 
        cube(8, center=true);
}


module xyz_to_angles(r, x, y, z) {
    translate([x, y, z]) {
        sphere(r=5);
    }

	dx = x - BODY_PIVOT_R * cos(r);
    dy = y - BODY_PIVOT_R * sin(r);
    dz = z;

    hip_angle = atan2(dy, dx) - r;

    // echo(rotation=rotation, x=x, y=y, z=z, 
    // 	dx=dx, dy=dy, dz=dz, hip_angle=hip_angle, dx2=dx2, dy2=dy2);
    
    dx2 = dx - HIP_L * cos(r + hip_angle);
    dy2 = dy - HIP_L * sin(r + hip_angle);

    // echo(rotation=rotation, x=x, y=y, z=z, 
    // 	dx=dx, dy=dy, dz=dz, hip_angle=hip_angle, dx2=dx2, dy2=dy2);
    leg_foot_ext = sqrt(dx2 * dx2 + dy2 * dy2 + dz * dz);
    // # leg_foot_ext is linear distance from hip pivot to foot tip (long side of iso triangle with ankle_angle as center)
    // # law of cosines to the rescue
    if (leg_foot_ext > LEG_L + FOOT_L)
        echo("cannot reach", leg_foot_ext);
    // # ankle = 180 - math.acos((LEG_L * LEG_L + FOOT_L * FOOT_L - leg_foot_ext * leg_foot_ext) / (2 * LEG_L * FOOT_L)) * RAD2DEG
    ankle_prime = acos(
    	(LEG_L * LEG_L + 
    		FOOT_L * FOOT_L - 
    		leg_foot_ext * leg_foot_ext) / (
    		2.0 * LEG_L * FOOT_L));
    
    // print(f"{locals()}")
    hip_foot_angle = -asin(dz / leg_foot_ext);

    /*
    (knee - hfa) + ankle_prime + toe_angle = 180;
    
    */

    /*
	180 = 2 * (hip_foot_angle - knee_angle) + ankle
	(180 - ankle) / 2 = hip_foot_angle - knee_angle
	knee_angle = hip_foot_angle - (180 - ankle)/2
    */
    knee_angle = min(
        max(
            hip_foot_angle - 90 + ankle_prime/2,
            -90), 90);
    // knee_angle = (ankle/2 - hip_foot_angle);




    // ankle_angle = min(90, max(-90, ankle + ankle_bias));
    ankle_angle = 180 - ankle_prime - ankle_bias;

    echo("xyz_to_a:", r=r, 
    	leg_foot_ext=leg_foot_ext,
        hip_angle=hip_angle, 
        knee_angle=knee_angle, 
        ankle_angle=ankle_angle,
    	x=x, y=y, z=z, 
    	dx=dx, dy=dy, dz=dz, dx2=dx2, dy2=dy2, 
    	ankle_prime=ankle_prime, hip_foot_angle=hip_foot_angle);

    translate([
            BODY_PIVOT_R * cos(r), 
            BODY_PIVOT_R * sin(r), 0]) {
    	rotate([0, 0, r + hip_angle]) {
    		cube([HIP_L, 1, 5]);
    		translate([HIP_L, 0, 0]) {
                rotate([0, hip_foot_angle, 0])
                    % cube([leg_foot_ext, 1, 5]);

    			rotate([0, knee_angle, 0]) {
    				cube([LEG_L, 1, 5]);
    				translate([LEG_L, 0, 0]) {
    					rotate([0, ankle_angle + ankle_bias, 0]) {
    						cube([FOOT_L, 1, 5]);
    					}

                        rotate([0, 180 - ankle_prime, 0]) {
                            % cube([FOOT_L, 1, 1]);
                        }
    				}

    			}
    		}
    	}
    }
}

for (r=[0:60:300]) {
    angles_to_xyz(r, hip, knee, ankle);
    xyz_to_angles(
        r=r,
        x=angles_to_x(r, hip, knee, ankle),
        y=angles_to_y(r, hip, knee, ankle),
        z=angles_to_z(r, hip, knee, ankle));
}