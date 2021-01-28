include <ds3225.scad>;
include <openbeam.scad>;
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

hip_angle = 60;
knee_angle = 30;
ankle_angle = 0;

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
		translate(hexhip_servo_offs)
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

hexbody_or = 45;
hexbody_pivot_r = 140;
hexbody_limit = 360;
hexbody_riser_l = 20;
hexbody_riser_w = 24;
hexbody_riser_h = ds3225_flange_z;

hexbody_side_l = 100;

hexrib_w = 100;
hexrib_l = 50;

hexrib_h = tab_th;
hexrib_offs = 5;
hexrib_hole_ir = 3.2/2;

module hexrib() {
	difference() {
		union() {
			translate([-hexrib_offs, 0, -hexrib_h]) {
				cylinder(r=hexrib_l/2, h=hexrib_h);
			}
			intersection() {
				translate([-hexrib_offs, 0, -2*hexrib_h]) {
					cylinder(r=hexrib_l/2, h=hexrib_h);
				}
				# translate([0, -25, -25])
					cube([50, 50, 50]);
			}
			intersection() {
				translate([-hexrib_offs*3 - hexrib_l/2, 0, -hexrib_h])
					cylinder(r=hexrib_w/2, h=hexrib_h, $fn=6);
				translate([
						0, 
						0, -hexrib_h])
					rotate([0, 0, 30])
					cylinder(r=hexrib_l, h=hexrib_h, $fn=6);
				
			}
			translate([-10, 0, -hexrib_h]) {
				cube([40, 26, 2*hexrib_h], center=true);
			}

			// * for (i=[-1, 1]) {
			// 	translate([-12, -20*i, -4])
			// 		rotate([0, 0, -30*i])
			// 		cube([18, 10, 2], center=true);
			// }
		}
		ds3225_horn_to_flange() {
			ds3225();
			ds3225_holes();
		}

		for (i=[-1, 1]) {
			for (dr=[0:15:15]) {
				translate([-10 + -i * (dr * sin(i*30)), i * (20 + dr * cos(i*30)), 0])
					cylinder(r=hexrib_hole_ir, h=hexrib_h*4, center=true, $fn=32);
			}
		}
		translate([-hexbody_pivot_r, 0, -openbeam_w - 2*hexrib_h])
			rotate([0, 0, 30])
			hexframe();
		servo_cap();
	}
}

! hexrib();

center_or = 40;

module hexbody_center() {
	difference() {
		translate([0, 0, tab_th-min_th])
			cylinder(r=center_or, h=tab_th, $fn=6);
		for (r=[0:60:360]) {
			for (dx=[5, 20]) {
				rotate([0, 0, r])
					translate([center_or - dx, 0, -1])
					cylinder(r=hexrib_hole_ir, h=tab_th+2, $fn=32);
			}
		}
		rotate([0, 0, 30])
			cylinder(r=15, h=tab_th+2, $fn=6);
		# hexframe();
	}
}
// ! hexbody_center();

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

hexframe_l = 200;
hexframe_w = 100;
hexframe_r = 100;

module hexframe() {
	for (r=[0:60:hexbody_limit]) {
		rotate([0, 0, r]) {
			translate([hexframe_r/2 + openbeam_w, 0, openbeam_w/2 + hexrib_h])
				rotate([0, 90, 0])
				openbeam(hexframe_r);
			translate([hexframe_r +3*openbeam_w/2, 0, 11.5])
				rotate([90, 0, 0])
				openbeam(hexframe_w);
			// translate([0, i*hexframe_w/2, openbeam_w/2 + hexrib_h])
			// 	rotate([90, 0, 0])
			// 	openbeam(hexframe_w);
		// translate([-i * 30*sin(30), i * 30*cos(30), openbeam_w/2 + hexrib_h])
		// 	rotate([0, 90, -60])
		// 	openbeam(50);
		}
	}
}

// !hexframe();

module hexbody() {
	// hexbody_bottom();
	% hexframe();
	// for (r=[60, 120, -60, -120]) {
	// 	rotate([0, 0, r])
	// 	translate([70, 0, 0])
	// 	rotate([0, 90, 0])
	// 	openbeam(100);
	// }
	hexbody_center();
	for (r=[30:60:hexbody_limit]) {
		rotate([0, 0, r])
		translate([hexbody_pivot_r, 0, 0]) 
		rotate([180, 0, 0]) {
			ds3225_horn_to_flange() {
				ds3225(hip_rotation(), show_model=true)
					hexhip_design();
			}
			servo_cap();
			hexrib();
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
				ds3225(hip_rotation(), show_model=true)
					hexhip_design_stl();
			}
			servo_cap_stl();
		}
	}
}

// hexbody_stl();
