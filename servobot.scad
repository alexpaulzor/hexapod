include <ds3225.scad>;

$t = 0.25;

axle_or = 5.2/2;

hip_rom = 120;
knee_bias = -90;
ankle_bias = 45;
knee_rom = 180;
foot_rom = 180;

// function hip_rotation() = hip_rom * abs(sin(180*$t)) - hip_rom/2;
// function knee_rotation() = -knee_rom * abs(sin(180*$t*2)) + knee_rom/2 + knee_bias;
// function foot_rotation() = foot_rom * abs(sin(180*$t*4)) - foot_rom/2;

hip_angle = 10;
knee_angle = -30;
ankle_angle = -90;

function hip_rotation() = hip_angle;
function knee_rotation() = knee_angle + knee_bias;
function foot_rotation() = ankle_angle + ankle_bias;


tab_th = 3;
min_th = 2;

// hexfoot_l = 100;
// hexfoot_h = 8;
// hexfoot_w = 15;
// hexleg_d = 24;
hexfoot_w = 50;
hexfoot_l = 100;
hexfoot_h = ds3225_horn_h + min_th;
hexfoot_d = 20;

hexfoot_bridge_l = 45;

module hexfoot() {
	difference() {
		union() { 
			translate([0, -hexfoot_d/2, 0])
				cube([hexfoot_bridge_l, hexfoot_d, hexfoot_h]);
			translate([0, -hexfoot_d/2, -hexfoot_w])
				cube([hexfoot_bridge_l, hexfoot_d, tab_th]);
			translate([0, 0, 6.2])
				cylinder(r=hexfoot_d/2, h=min_th, $fn=36);
			translate([0, 0, -hexfoot_w])
				cylinder(r=hexfoot_d/2, h=tab_th, $fn=36);
			translate([hexfoot_bridge_l, -hexfoot_d/2, -hexfoot_w])
				cube([min_th, hexfoot_d, hexfoot_w + hexfoot_h]);
			* translate([hexfoot_l, 0, -2])
				ds3225_center_to_horn()
				cube([ds3225_flange_l, hexfoot_d, 20], center=true);
			translate([
					hexfoot_bridge_l + (hexfoot_l - hexfoot_bridge_l)/2, 
					0, -hexfoot_w/2 + tab_th/2])
				rotate([0, 90, 0])
				translate([-ds3225_horn_h/2 + 0.4, 0, 0])
				intersection() {
					cylinder(r1=(hexfoot_w+hexfoot_h)/2 + tab_th, r2=0, h=hexfoot_l - hexfoot_bridge_l, center=true);
					cube([(hexfoot_w + hexfoot_h), hexfoot_d, 100], center=true);
				}
			// translate([hexhip_l - 20, 0, -ds3225_axle_flange_offset - 3])
			* translate([hexfoot_l, -1, -33])
				cube([20, 22, 20], center=true);
			
		}
		ds3225_horn();
		translate([0, 0, hexfoot_h])
			ds3225_horn_holes();
		translate([0, 0, hexfoot_h/4])
			ds3225_horn_holes();
		translate([0, 0, -ds3225_flange_l])
			cylinder(r=5/2, h=ds3225_flange_l*2, $fn=36);
		* translate([hexfoot_l, 0, 0])
			rotate([0, 0, 0]) {
				ds3225(show_model=false);
				ds3225_holes();
				translate([0, 10, -2])
					ds3225_center_to_horn()
					cube([ds3225_l-10, 10, 20+1], center=true);
				translate([0, 0, -ds3225_flange_l])
					cylinder(r=axle_or, h=ds3225_flange_l*2, $fn=36);
			}
		translate([
					hexfoot_bridge_l + (hexfoot_l - hexfoot_bridge_l)/2, 
					0, -hexfoot_w/2 + tab_th/2])
				rotate([0, 90, 0])
				translate([-ds3225_horn_h/2 + 0.4, 0, 0])
					cylinder(r1=(hexfoot_w+hexfoot_h)/2 - tab_th, r2=0, h=(hexfoot_l - hexfoot_bridge_l)-6, center=true);
		// translate([0, 0, -50])
		// 	cube([100, 100, 100]);
	}
}

// !hexfoot();

module hexfoot_stl() {
	import("stl/hexfoot.servobot.stl");
}

hexleg_w = 50;
hexleg_l = 100;
hexleg_h = ds3225_horn_h + min_th;
hexleg_d = 24;

hexleg_bridge_l = 45;

module hexleg() {
	difference() {
		union() { 
			translate([0, -hexleg_d/2, 0])
				cube([hexleg_bridge_l, hexleg_d, hexleg_h]);
			translate([0, -hexleg_d/2, -hexleg_w])
				cube([hexleg_bridge_l, hexleg_d, tab_th]);
			translate([0, 0, ds3225_horn_h])
				cylinder(r=hexleg_d/2, h=min_th, $fn=36);
			translate([0, 0, -hexleg_w])
				cylinder(r=hexleg_d/2, h=tab_th, $fn=36);
			translate([hexleg_bridge_l, -hexleg_d/2, -hexleg_w])
				cube([tab_th, hexleg_d, hexleg_w + hexleg_h]);
			translate([hexleg_l, 0, -2])
				ds3225_center_to_horn()
				cube([ds3225_flange_l, hexleg_d, 20], center=true);
			translate([hexleg_bridge_l + 10, 0, -hexleg_w/2 + 2.4])
				cube([20, hexleg_d, 20], center=true);
			// translate([hexhip_l - 20, 0, -ds3225_axle_flange_offset - 3])
			translate([hexleg_l, -1, -35])
				cube([20, 22, 20], center=true);
			
		}
		ds3225_horn();
		translate([0, 0, hexleg_h])
			ds3225_horn_holes();
		translate([0, 0, hexleg_h/3])
			ds3225_horn_holes();
		translate([0, 0, -ds3225_flange_l])
			cylinder(r=5/2, h=ds3225_flange_l*2, $fn=36);
		translate([hexleg_l, 0, 0])
			rotate([0, 0, 0]) {
				ds3225(show_model=false);
				ds3225_holes();
				translate([0, 10, -2])
					ds3225_center_to_horn()
					cube([ds3225_l-10, 10, 20+1], center=true);
				translate([0, 0, -ds3225_flange_l])
					cylinder(r=axle_or, h=ds3225_flange_l*2, $fn=36);
				
			}
		translate([hexleg_bridge_l + 9, 0, -hexleg_w/2 + 2.4])
			rotate([90, 0, 0])
			cylinder(r=7, h=26, center=true);
		hexleg_brace();
	}
	// translate([hexleg_l, 0, 0])
	// rotate([0, 0, 0])
	// % ds3225(knee_rotation());
}
// ! hexleg();

module hexleg_stl() {
	import("stl/hexleg.servobot.stl");
}

module hexleg_brace() {
	difference() {
		translate([hexleg_l, min_th, -2])
			ds3225_center_to_horn() 
			cube([ds3225_flange_l, hexleg_d - 2*min_th, 10], center=true);
		translate([hexleg_l, 0, -2]) {
			ds3225_holes();
			ds3225();
		}
	}
}

// !hexleg_brace();

module hexleg_brace_stl() {
	import("stl/hexleg_brace.servobot.stl");
}

module hexleg_design() {
	hexleg();
	hexleg_brace();
	% ds3225_horn();
	translate([hexleg_l, 0, 0])
		rotate([0, 0, 0]) {
			ds3225(foot_rotation(), show_model=true) {
				hexfoot();
			}
			wire_clip();
		}
}

// ! hexleg_design();

module hexleg_design_stl() {
	hexleg_stl();
	hexleg_brace_stl();
	% ds3225_horn_stl();
	translate([hexleg_l, 0, 0])
		rotate([0, 0, 0])
		ds3225(foot_rotation(), show_model=true) {
			hexfoot_stl();
		}
}

hexhip_l = 50;
hexhip_w = 20;
hexhip_h = ds3225_horn_h + min_th;;
hexhip_d = ds3225_w + min_th * 2;
hexhip_z = 50;
hexhip_servo_x = hexhip_l - ds3225_w/2 - min_th;
hexhip_servo_offs = [hexhip_servo_x, -23, -10];

echo(hexhip_servo_offs=hexhip_servo_offs);

module hexhip() {
	difference() {
		union() {
			translate([0, -hexhip_w/2, 0]) {
				cube([hexhip_l, hexhip_w, hexhip_h]);
			}
			translate([0, 0, hexhip_h - min_th])
				cylinder(r=hexhip_w/2, h=min_th);
			translate([
					hexhip_l - hexhip_d, 
					-hexhip_w/2, 
					-hexhip_z])
				cube([
					hexhip_d, 
					hexhip_w, 
					ds3225_flange_l]);

			translate([0, -hexhip_w/2, -hexhip_z]) {
				cube([hexhip_l, hexhip_w, tab_th]);
			}
			translate([0, 0, -hexhip_z]) 
				cylinder(r=hexhip_w/2, h=tab_th);
			translate([hexhip_l - 20, 0, -ds3225_axle_flange_offset - 3])
				cube([20, 22, 20]);
		}
		translate([0, 0, -ds3225_flange_l])
			cylinder(r=axle_or, h=ds3225_flange_l*2, $fn=36);
		* translate([0, -hexhip_w/2, -ds3225_flange_l + hexhip_h + min_th])
			cube([hexhip_l - hexhip_d, hexhip_w, min_th]);
		// # translate([-5, 0, 0])
		// 	ds3225_horn(); 
		ds3225_horn();
		translate([0, 0, hexhip_h-2])
			ds3225_horn_holes();
		translate([0, 0, 3])
			ds3225_horn_holes();
		translate([hexhip_servo_x, -hexleg_w/2, -10])
			rotate([90, -90, 0])
			ds3225_holes();
		# translate(hexhip_servo_offs)
			rotate([90, -90, 0]) {
				ds3225(knee_rotation(), show_model=false);
				translate([0, 0, -ds3225_flange_l])
					cylinder(r=axle_or, h=ds3225_flange_l*2, $fn=36);
			}

		translate([25, 0, -ds3225_l/2])
			cube([10, hexhip_w+1, ds3225_l-10], center=true);
	}
}

// ! hexhip();

module hexhip_stl() {
	import("stl/hexhip.servobot.stl");
}

module hexhip_design() {
	hexhip();
	ds3225_horn();
	translate(hexhip_servo_offs)
		rotate([90, -90, 0]) {
			ds3225(knee_rotation(), show_model=true) 
			hexleg_design();
			wire_clip();
		}
	// % translate([0, 0, -ds3225_horn_dz])
	// 	ds3225(show_model=true);
	* translate([0, 0, -ds3225_h +ds3225_horn_dz + ds3225_flange_z - tab_th]) 
		servo_cap();
	// # translate(hexhip_servo_offs)
	// 	translate([-10, 0, -hexhip_l + wire_clip_h + tab_th + min_th])
	// 	wire_clip();
}

// !hexhip_design();

module hexhip_design_stl() {
	hexhip_stl();
	% ds3225_horn_stl();
	translate(hexhip_servo_offs)
		rotate([90, -90, 0])
		ds3225(knee_rotation(), show_model=true) 
		hexleg_design_stl();
	// % translate([0, 0, -ds3225_horn_dz])
	// 	ds3225(show_model=true);
	// * translate([0, 0, -ds3225_h +ds3225_horn_dz + ds3225_flange_z - tab_th]) 
	// 	servo_cap_stl();
}

wire_clip_w = 20 + 2 * min_th;
wire_clip_h = (ds3225_flange_l - ds3225_l)/2 + min_th;

module wire_clip() {

	difference() {
		union() {
			translate([
				ds3225_axle_flange_offset - ds3225_flange_l - min_th, 
				-wire_clip_w/2, 
				-min_th - 20])
				ds3225_horn_to_flange(-1)
				cube([wire_clip_h, wire_clip_w, min_th]);
			translate([
				ds3225_axle_flange_offset - ds3225_flange_l - min_th, 
				-wire_clip_w/2, 
				-min_th - 9])
				ds3225_horn_to_flange(-1)
					scale([1, 0.4, 1])
					rotate([0, 90, 0])
					cylinder(r=11, h=wire_clip_h);
		}
		ds3225_holes();
		translate([
			ds3225_axle_flange_offset - ds3225_flange_l - min_th, 
			-wire_clip_w/2, 
			-min_th - 9])
			ds3225_horn_to_flange(-1)
				scale([1, 0.4, 1])
				rotate([0, 90, 0])
				cylinder(r=11-min_th, h=wire_clip_h);
		translate([
				ds3225_axle_flange_offset - ds3225_flange_l - min_th, 
				-wire_clip_w/2, 
				-20])
				ds3225_horn_to_flange(-1)
				cube([wire_clip_h, wire_clip_w, 20]);
			
	}
	// % ds3225(show_model=true);
	// % hexhip_design();
}

// ! wire_clip();

hexbody_or = 125;
hexbody_pivot_r = 100;
hexbody_limit = 30; //360;
hexbody_riser_l = 20;
hexbody_riser_w = 24;
hexbody_riser_h = ds3225_flange_z;

module hexbody_wafer(include_ds3225=true) {
	difference() {
		union() {
			rotate([0, 0, 0])
				cylinder(r=hexbody_or, h=tab_th, $fn=6);
			for (r=[0:60:hexbody_limit]) {
				rotate([0, 0, r]) {
					for (i=[-1, 1])
					translate([0, -tab_th/2 + i*(ds3225_w/2 + tab_th/2), -tab_th])
					cube([hexbody_or, tab_th, tab_th]);
				}
			}
		}
		for (r=[0:60:hexbody_limit]) {
			rotate([0, 0, r])
			translate([hexbody_pivot_r, 0, 0])
			rotate([180, 0, 0]) {
				ds3225_horn_to_flange() {
					if (include_ds3225) {
						ds3225(hip_rotation());
					}
					ds3225_holes();
					translate([0, 0, -ds3225_flange_l])
						cylinder(r=axle_or, h=ds3225_flange_l*2, $fn=36);
				}
				translate([ds3225_axle_flange_offset + 2, -20, -3*tab_th])
					cube([10, 40, 5*tab_th]);
			}
			rotate([0, 0, 30+r]) {
				translate([hexbody_or + 2, 0, -2*min_th])
				cylinder(r=hexbody_pivot_r/2, h=4*min_th);
				for (x=[15:15:60]) {
					translate([x, 0, -1])
					cylinder(r=3/2, h=min_th*2, $fn=36);
				}
				cylinder(r=3/2, h=min_th*2, $fn=36);
			}
		}
	}
}

module hexbody_bottom() {
	hexbody_wafer(true);
	// % //rotate([0, 0, 15])
	// 	cube([140, 140, 140], center=true);
}

// ! hexbody_bottom();

module hexbody_bottom_stl() {
	import("stl/hexbody_bottom.servobot.stl");
}

servo_cap_h = ds3225_flange_z;


module servo_cap() {
	corner_r = (ds3225_flange_l - ds3225_l) / 2 - min_th;
	difference() {
		ds3225_center_to_horn()
		ds3225_horn_to_flange() 
		union() {
			translate([
					-ds3225_l/2 - min_th, -10, 
					-servo_cap_h - ds3225_h/2 + ds3225_flange_z - tab_th])
				cube([ds3225_l + 2 * min_th, 
					20, servo_cap_h], center=false);
			translate([0, 0, -ds3225_h/2 + ds3225_flange_z - 1.5*tab_th])
				cube([ds3225_flange_l, 
					20-2*corner_r, tab_th], center=true);
			for (x=[-1, 1]) {
				for (y=[-1, 1]) {
					translate([
							x*(ds3225_flange_l/2 - corner_r), 
							y*(10-corner_r), -ds3225_h/2 + ds3225_flange_z - 1.5*tab_th])
						cylinder(r=corner_r, h=tab_th, center=true);
				}
			}
		}
		ds3225_horn_to_flange() {
			ds3225_holes();
			ds3225(show_model=true);
			ds3225(show_model=false);
		}
		translate([0, 0, -ds3225_flange_l])
			cylinder(r=5/2, h=ds3225_flange_l*2, $fn=36);
		translate([10, 0, -21.5])
			cube([8, 8, 8], center=true);
	}

	* ds3225_horn_to_flange()
		ds3225(show_model=true);
}

// !servo_cap();

module servo_cap_stl() {
	import("stl/servo_cap.servobot.stl");
}

module hexbody() {
	hexbody_bottom();
	* translate([0, 0, ds3225_flange_z])
		hexbody_top();
	for (r=[0:60:hexbody_limit]) {
		rotate([0, 0, r])
		translate([hexbody_pivot_r, 0, 0]) 
		rotate([180, 0, 0]) {
			ds3225_horn_to_flange() {
				ds3225((r % 120 == 0 ? 1 : -1) * hip_rotation(), show_model=true)
					hexhip_design();
			}
			servo_cap();
		}
	}
}

// 	hexleg();
hexbody();

module hexbody_stl() {
	hexbody_bottom_stl();
	for (r=[0:60:hexbody_limit]) {
		rotate([0, 0, r])
		translate([hexbody_pivot_r, 0, 0]) 
		rotate([180, 0, 0]) {
			ds3225_horn_to_flange() {
				ds3225((r % 120 == 0 ? 1 : -1) * hip_rotation(), show_model=true)
					hexhip_design_stl();
			}
			servo_cap_stl();
		}
	}
}

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
}
angles_to_xyz();

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

    translate([hexbody_pivot_r * cos(rotation), 0, 0]) {
    	rotate([0, 0, hip_angle]) {
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
    // if leg.ankle_angle < -90:
    //     leg.ankle_angle = -90
    // if leg.ankle_angle > 90:
    //     leg.ankle_angle = 90
    
	/*
	dx = leg.x - BODY_PIVOT_R * math.cos(DEG2RAD * (leg.rotation));
    dy = leg.y - BODY_PIVOT_R * math.sin(DEG2RAD * (leg.rotation));
    dz = leg.z - HIP_DZ;

    leg.hip_angle = math.atan2(dy, dx) * RAD2DEG - leg.rotation;

    dx -= HIP_L * math.cos(DEG2RAD * (leg.rotation + leg.hip_angle))
    dy -= HIP_L * math.sin(DEG2RAD * (leg.rotation + leg.hip_angle))

    leg_foot_ext = math.sqrt(dx * dx + dy * dy + dz * dz);
    # leg_foot_ext is linear distance from hip pivot to foot tip (long side of iso triangle with ankle_angle as center)
    # law of cosines to the rescue
    if leg_foot_ext > LEG_L + FOOT_L:
        print(f"cannot reach {locals()}")


    # ankle = 180 - math.acos((LEG_L * LEG_L + FOOT_L * FOOT_L - leg_foot_ext * leg_foot_ext) / (2 * LEG_L * FOOT_L)) * RAD2DEG
    ankle = math.acos((LEG_L * LEG_L + FOOT_L * FOOT_L - leg_foot_ext * leg_foot_ext) / (2 * LEG_L * FOOT_L)) * RAD2DEG
    
    print(f"{locals()}")
    hip_foot_angle = math.asin(dz / leg_foot_ext) * RAD2DEG
    leg.knee_angle = (ankle/2 - hip_foot_angle) - 90

    # if leg.knee_angle < -90 or leg.knee_angle > 90:
    #     # knee wont reach, so invert
    #     print(f"inverting {locals()}")
    #     # print("inverting")
    #     ankle = -ankle
    #     leg.knee_angle = -(ankle/2 - hip_foot_angle)


    leg.ankle_angle = ankle - ANKLE_BIAS
    if leg.ankle_angle < -90:
        leg.ankle_angle = -90
    if leg.ankle_angle > 90:
        leg.ankle_angle = 90
    print(f"{locals()}")
    
    return leg
    */
}

// ! xyz_to_angles(0, 338, 0, 0);
xyz_to_angles(0, 248.198, -26.1313, 146.593);

// hexbody_stl();
// */