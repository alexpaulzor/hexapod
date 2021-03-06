include <ds3225.scad>;
include <cast_parts.scad>;
include <openbeam.scad>;
include <metric.scad>;
$t = 0.25;

axle_or = 5.2/2;

hip_rom = 90;
knee_bias = -90;
ankle_bias = 45;
knee_rom = 180;
foot_rom = 180;

// function hip_rotation() = hip_rom * abs(sin(180*$t)) - hip_rom/2;
// function knee_rotation() = -knee_rom * abs(sin(180*$t*2)) + knee_rom/2 + knee_bias;
// function foot_rotation() = foot_rom * abs(sin(180*$t*4)) - foot_rom/2;

hip_angle = 45;
knee_angle = -70;
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
hexfoot_l = 170;
hexfoot_h = ds3225_horn_h + min_th;
hexfoot_d = 20;

hexfoot_bridge_l = 45;

hexfoot_frame_dz = -ds3225_h + ds3225_flange_z - openbeam_w/2 - tab_th;

module hexfoot_frame() {
	translate([hexleg_bridge_l + openbeam_w/2, 0, -25])
		openbeam(50);
	translate([hexleg_bridge_l + openbeam_w + 50, 0, hexfoot_frame_dz])
		rotate([0, 90, 0])
		openbeam(100);
	translate([hexleg_bridge_l + openbeam_w - 25, 0, -50 - openbeam_w/2])
		rotate([0, 90, 0])
		openbeam(50);
	
	translate([hexleg_bridge_l + openbeam_w/2, -openbeam_w/2-openbeam_bracket_th/2, -50 - openbeam_w/2])
		rotate([90, -90, 0])
		l_bracket();

	translate([hexleg_bridge_l + openbeam_w/2, openbeam_w/2 + openbeam_bracket_th/2, hexfoot_frame_dz])
	rotate([90, 90, 0])
		tee_bracket();
}

hexfoot_r = 15;

// module hexfoot_tip() {
// 	difference() {
// 	    sphere(r=hexfoot_r);
// 		translate([(hexfoot_r-2)/2, 0, 0])
// 			sphere(r=hexfoot_r/2);
// 		// translate([-hexfoot_r/2, 0, 0])
// 		// 	rotate([0, 90, 0]) 
// 		// 	openbeam(hexfoot_r*2);
// 		translate([-hexfoot_r, 0, 0])
// 			cube([hexfoot_r*3, openbeam_w + 0.2, openbeam_w + 0.2], center=true);
// 		rotate([90, 0, 0])
// 			cylinder(r=3.2/2, h=40, center=true, $fn=32);
// 		cylinder(r=3.2/2, h=40, center=true, $fn=32);
// 		for (r=[0:90:360]) {
// 			rotate([r, 0, 0])
// 				translate([0, 0, openbeam_w/2 + min_th])
// 				cylinder(
// 					r=9/2, h=hexfoot_r - openbeam_w/2 - min_th, $fn=32);
// 		}
// 		translate([-hexfoot_r, 0, 0])
// 			cube([hexfoot_r/2, 2*hexfoot_r, 2*hexfoot_r], center=true);
// 	}
// }

// ! hexfoot_tip();

module hexfoot() {
	hexleg_fork_horn();
	hexleg_fork_pivot();
	% hexfoot_frame();
	// translate([hexfoot_l - hexfoot_r, 0, -25])
	// 	hexfoot_tip();

	
	
	// rotate([0, 0, 0])
	// % ds3225(knee_rotation());
}

// !hexfoot();

hexleg_w = 50;
hexleg_l = 150;
hexleg_h = ds3225_horn_h + min_th;
hexleg_d = 24;

hexleg_motor_dy = openbeam_w/2 + ds3225_w/2;

hexleg_bridge_l = 48;

module hexleg_frame() {
	translate([hexleg_bridge_l + openbeam_w/2, 0, -25])
		openbeam(50);
	translate([hexleg_bridge_l + openbeam_w + 50, 0, hexfoot_frame_dz])
		rotate([0, 90, 0])
		openbeam(100);
	translate([hexleg_bridge_l + openbeam_w - 25, 0, -50 - openbeam_w/2])
		rotate([0, 90, 0])
		openbeam(50);
	
	translate([hexleg_bridge_l + openbeam_w/2, -openbeam_w/2-openbeam_bracket_th/2, -50 - openbeam_w/2])
		rotate([90, -90, 0])
		l_bracket();

	translate([hexleg_bridge_l + openbeam_w/2, openbeam_w/2 + openbeam_bracket_th/2, hexfoot_frame_dz])
	rotate([90, 90, 0])
		tee_bracket();
}

hexleg_fork_shift = 8;
hexleg_fork_d = openbeam_w + 2*min_th;

module hexleg_fork_horn() {
	difference() {
		// union() { 
			// intersection() {
			// 	* translate([-hexleg_fork_shift, 0, hexleg_fork_d/4]) {
			// 		rotate([0, 90, 0])
			// 		cylinder(
			// 			r=hexleg_fork_d/2, 
			// 			h=hexleg_bridge_l + hexleg_fork_shift + openbeam_w + min_th);
			// 	}
				translate([-openbeam_w, -hexleg_fork_d/2, 0])
					cube([hexleg_bridge_l + 2*openbeam_w + min_th, 
					hexleg_fork_d, openbeam_w]);
			// }
			// * translate([hexleg_bridge_l, -hexleg_fork_d/2, -openbeam_w])
			// 	cube([openbeam_w, min_th, openbeam_w + hexleg_fork_d/4]);
			
			// * translate([hexleg_bridge_l + openbeam_w, -hexleg_fork_d/2, -openbeam_w])
			// 	cube([min_th, hexleg_fork_d, openbeam_w + hexleg_fork_d/4]);
			// * translate([hexleg_bridge_l - min_th, -hexleg_fork_d/2, -openbeam_w])
			// 	cube([min_th, hexleg_fork_d, openbeam_w + hexleg_fork_d/4]);
		// }

		translate([hexleg_bridge_l + openbeam_w/2, 0, -3*openbeam_w/2])
			rotate([90, 0, 90])
			cylinder(r=3.2/2, h=30, center=true, $fn=32);
		translate([hexleg_bridge_l + openbeam_w/2, 0, -openbeam_w/2])
			rotate([90, 0, 90])
			cylinder(r=3.2/2, h=30, center=true, $fn=32);
		translate([hexleg_bridge_l + openbeam_w/2, 0, -openbeam_w/2])
			rotate([90, 0, 0])
			cylinder(r=3.2/2, h=30, center=true, $fn=32);
		
		translate([hexleg_bridge_l + openbeam_w/2, 0, tab_th])
			cylinder(r=4.5, h=openbeam_w, $fn=32);
		for (dx=[0, -15, -30])
			translate([hexleg_bridge_l + openbeam_w/2 + dx, 0, openbeam_w/2])
			rotate([90, 0, 0])
			cylinder(r=3.2/2, h=30, center=true, $fn=32);
		translate([hexleg_bridge_l + openbeam_w/2, 0, 1])
			cylinder(r=3.2/2, h=30, center=true, $fn=32);
		cast_ds3225_horn();
		ds3225_horn_holes(30);
		translate([0, 0, hexhip_w/2 + 5])
				// hexhip_h-2 + ds3225_horn_h + min_th])
			ds3225_horn_holes(h=10, r=5);
		
		// # cylinder(r=18/2, h=30);
		// hexleg_frame();
		
	}
}
// ! cast_ds3225_horn();

!hexleg_fork_horn();

hexleg_fork_pivot_l = 40;

module hexleg_fork_pivot() {
	difference() {
		union() { 
			translate([0, -openbeam_w/2, -hexleg_w])
				cube([hexleg_fork_pivot_l, openbeam_w, tab_th]);
			translate([0, 0, -hexleg_w])
				cylinder(r=openbeam_w/2, h=tab_th, $fn=36);
			translate([0, openbeam_w/2, -hexleg_w - openbeam_w])
				cube([hexleg_fork_pivot_l, min_th, openbeam_w + tab_th]);
			translate([
					hexleg_bridge_l + openbeam_w - 50 - min_th, -openbeam_w/2, -hexleg_w - openbeam_w])
				cube([min_th, openbeam_w, openbeam_w + tab_th]);	
		}
		translate([0, 0, -ds3225_flange_l])
			cylinder(r=5/2, h=ds3225_flange_l*2, $fn=36);
		translate([-4, 0, -ds3225_flange_l - tab_th - openbeam_w/2])
			rotate([90, 0, 0])
				cylinder(r=openbeam_w, h=2*openbeam_w, center=true, $fn=36);
		translate([0, 0, -ds3225_flange_l - tab_th])
			rotate([0, 90, 0])
				cylinder(r=3.2/2, h=2*openbeam_w, center=true, $fn=36);
			
		for (x=[18:openbeam_w:hexleg_fork_pivot_l]) {
			translate([x, 0, -ds3225_flange_l - tab_th]) {
				cylinder(r=3.2/2, h=2*openbeam_w, center=true, $fn=36);
				rotate([90, 0, 0])
				cylinder(r=3.2/2, h=2*openbeam_w, center=true, $fn=36);
			}

		}
	}
}

module motor_plate() {
	difference() {
		union() {
			translate([ds3225_axle_flange_offset - ds3225_flange_l, 
				-openbeam_w/2, -ds3225_h + ds3225_flange_z - tab_th])
				// -25 + openbeam_w/2])
				cube([ds3225_flange_l, 
					ds3225_w + openbeam_w + tab_th, tab_th]);
			translate([
					ds3225_axle_flange_offset - 4, 
					-openbeam_w/2, -25 - openbeam_w/2])
				cube([4, 
					openbeam_w, 
					openbeam_w]);
		}

		translate([0, hexleg_motor_dy, 0])
			rotate([0, 0, 0]) {
				ds3225(foot_rotation(), show_model=false);
				ds3225_holes();
			}
		for (dx=[-25:15:15])
			translate([dx, 0, -25])
			cylinder(r=3.2/2, h=30, center=true, $fn=32);
		translate([15, 0, -25])
			rotate([0, 90, 0])
			cylinder(r=3.2/2, h=30, center=true, $fn=32);
	
	}

}

module pivot_plate() {
	difference() {
		union() {
			translate([
					-openbeam_w/2, -openbeam_w/2, 
					-ds3225_h - 4.5])
				cube([
					openbeam_w + 5.7, openbeam_w + ds3225_w, 4.5]);
			translate([
					-openbeam_w/2, -openbeam_w/2, 
					-ds3225_h - 4])
				cube([
					openbeam_w + 5.7, openbeam_w, ds3225_flange_z - openbeam_w]);
			translate([
					-openbeam_w/2, -openbeam_w/2 - min_th, 
					-25 - openbeam_w/2 - tab_th])
				cube([
					openbeam_w + 5.7, min_th, openbeam_w + tab_th]);
		
		}
		translate([
				0, -tab_th, 
				-25 -openbeam_w - min_th])
			rotate([0, 90, 0])
			cylinder(r=openbeam_w/2, h=50, center=true, $fn=32);
		translate([
				-openbeam_w/2-1, -openbeam_w/2 - tab_th, 
				-25 -2*openbeam_w])
			cube([2*openbeam_w+2, openbeam_w, openbeam_w]);
		translate([
				-openbeam_w/2-1, -openbeam_w, 
				-25 -3*openbeam_w/2 - min_th])
			cube([2*openbeam_w+2, openbeam_w, openbeam_w]);
		translate([0, 0, -25])
			rotate([90, 0, 0])
			cylinder(r=3.2/2, h=30, center=true, $fn=32);
		
		translate([0, 0, -25 - openbeam_w - min_th])
			cylinder(r=4.5, h=openbeam_w, center=true, $fn=32);
		translate([0, 0, -2*openbeam_w])
			cylinder(r=3.2/2, h=30, center=true, $fn=32);
		# translate([0, hexleg_motor_dy, -ds3225_h - m5_screw_cap_h])
			rotate([180, 0, 0])
			m5_screw();
	}
	
}

module hexleg() {
	hexleg_fork_horn();
	hexleg_fork_pivot();

	% hexleg_frame();
	translate([hexleg_l, 0, 0])
		motor_plate();
	translate([hexleg_l, 0, 0])
		pivot_plate();
	// rotate([0, 0, 0])
	// % ds3225(knee_rotation());
}
// ! hexleg();

/*
module hexleg_brace() {
	difference() {
		translate([hexleg_l, hexleg_d/4, -2])
			ds3225_center_to_horn() 
			cube([ds3225_flange_l, 
				hexleg_d/2 + min_th, 
				10], center=true);
		translate([hexleg_l, 0, -2]) {
			ds3225_holes();
			ds3225();
		}
	}
}
*/
// !hexleg_brace();

module hexleg_design() {
	hexleg();
	// hexleg_brace();
	% ds3225_horn();
	translate([hexleg_l, hexleg_motor_dy, 0])
		rotate([0, 0, 0]) {
			ds3225(foot_rotation(), show_model=true) {
				hexfoot();
			}
			// wire_clip();
		}
}

! hexleg_design();

hexhip_l = 50;
hexhip_w = 20;
hexhip_h = ds3225_horn_h + min_th;;
hexhip_d = ds3225_w + min_th;
hexhip_z = 50;
hexhip_servo_x = hexhip_l - ds3225_w/2 - min_th;
hexhip_servo_offs = [hexhip_servo_x, -23, -10];

echo(hexhip_servo_offs=hexhip_servo_offs);

module hexhip() {
	// Bonus TOOD: make the leg brace also only top screw
	// TODO: channel for wires

	// TODO: use captive screw instead of simple hole
	difference() {
		union() {
			translate([0, -hexhip_w/2, 0]) {
				cube([hexhip_l, hexhip_w, hexhip_h]);
			}
			translate([0, 0, hexhip_w/2 - tab_th]) {
				rotate([0, 90, 0])
				cylinder(r=hexhip_w/2, h=hexhip_l);
				sphere(r=hexhip_w/2);
			}
			// # translate([0, 0, hexhip_h - min_th])
			// 	cylinder(r=hexhip_w/2, h=min_th);
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
			translate([0, 0, -hexhip_z + tab_th]) {
				rotate([0, 90, 0])
				cylinder(r1=hexhip_w/2, r2=hexhip_w/4, h=hexhip_l);
				sphere(r=hexhip_w/2);
			}
			translate([0, 0, -hexhip_z]) 
				cylinder(r=hexhip_w/2, h=tab_th);
			*translate([hexhip_l - 20, 0, -ds3225_axle_flange_offset - 3])
				cube([20, 22, 20]);
		}
		hexhip_brace();
		// # for (dx=[0:2:10]) {
		// 	translate([-dx, 0, 0])
		// 		hexhip_brace();
		// }
		// translate([hexhip_servo_x, 5, 
				// -ds3225_flange_l/2 - ds3225_l/2 + 0.8])
			// rotate([90, -90, 0]) 
			// ds3225_center_to_horn() 
		hexhip_brace_offs()
			translate([0, 0, -0.5])
			cube([
				ds3225_flange_l, 
				hexhip_servo_x/2 + 10 + min_th, 
				ds3225_horn_w+1]);
		translate([0, 0, -ds3225_flange_l/2])
			cylinder(r=axle_or, h=ds3225_flange_l*2, center=true, $fn=36);
		translate([0, 0, hexhip_w/2])
			cylinder(r=axle_or*2, h=hexhip_w/2, $fn=36);
		translate([0, 0, -hexhip_w/2-hexhip_z])
			cylinder(r=axle_or*2, h=hexhip_w/2, $fn=36);
		
		ds3225_horn();
		translate([0, 0, hexhip_h-2])
			ds3225_horn_holes(30);
		translate([0, 0, hexhip_w/2 + 5])
				// hexhip_h-2 + ds3225_horn_h + min_th])
			ds3225_horn_holes(h=10, r=5);
		translate([hexhip_servo_x, -hexleg_w/2, -10])
			rotate([90, -90, 0])
			ds3225_holes();
		translate(hexhip_servo_offs)
			rotate([90, -90, 0]) {
				ds3225(knee_rotation(), show_model=false);
				// % ds3225(knee_rotation(), show_model=true);
				translate([0, 0, -ds3225_flange_l])
					cylinder(r=axle_or, h=ds3225_flange_l*2, $fn=36);
			}
				
		translate([25, 0, -ds3225_l/2])
			cube([10, hexhip_w+1, ds3225_l-10], center=true);
		translate([0, 0, -hexhip_z + tab_th]) {
			cylinder(r=20, h=hexhip_w/2);
		}
		translate([0, 0, -hexhip_w/2]) {
			cylinder(r=24, h=hexhip_w/2);
		}
		translate([-hexhip_w/2, -hexhip_w/2, 0])
			cube([hexhip_w/2, hexhip_w, ds3225_horn_h]);
	}
}

// ! hexhip();

module hexhip_pivot() {
	difference() {
		translate([hexhip_servo_x - ds3225_w/2, 10, -ds3225_flange_l + 7.5])
			cube([ds3225_w, 12.5, ds3225_flange_l]);
		translate(hexhip_servo_offs)
			rotate([90, -90, 0]) {
				ds3225(knee_rotation(), show_model=false);
				% ds3225(knee_rotation(), show_model=true);
				ds3225_holes();
			}	
		translate([hexhip_servo_x - 4, 10, 0])
			cube([8, 5, 8]);
		translate([hexhip_servo_x - ds3225_w/2, 20, 0.5])
			cube([ds3225_w, 2.5, 7]);
		translate([hexhip_servo_x - ds3225_w/2, 20, -ds3225_flange_l + 7.5])
			cube([ds3225_w, 2.5, 7]);
		# translate([hexhip_servo_x, 20.7, -10])
			rotate([-90, 0, 0])
			m5_screw();
	}	

}

// !hexhip_pivot();

module hexhip_brace_offs() {
	translate([hexhip_servo_x, ds3225_horn_w/2, 
				-ds3225_flange_l/2 - ds3225_l/2 - 0.5])
			rotate([90, -90, 0])
			children();
}

module hexhip_brace() {
	difference() {
		// translate([hexhip_servo_x, 5, 
		// 		-ds3225_flange_l/2 - ds3225_l/2 + 0.8])
		// 	rotate([90, -90, 0]) 
			// ds3225_center_to_horn() 
			hexhip_brace_offs()
				cube([
				ds3225_flange_l, 
				10 + min_th, ds3225_horn_w]);
			// *cube([
			// 	ds3225_flange_l, 
			// 	10 + min_th, 
			// 	10], center=true);
		
		translate(hexhip_servo_offs)
			rotate([90, -90, 0]) {
			ds3225_holes();
			ds3225();
		}
		ds3225_horn();
		
	}
}

// !hexhip_brace();

module hexhip_design() {
	hexhip();
	hexhip_brace();
	hexhip_pivot();
	ds3225_horn();
	translate(hexhip_servo_offs)
		rotate([90, -90, 0]) {
			ds3225(knee_rotation(), show_model=true) 
			hexleg_design();
			// wire_clip();
		}
	// % translate([0, 0, -ds3225_horn_dz])
	// 	ds3225(show_model=true);
	% translate([0, 0, -ds3225_h +ds3225_horn_dz + ds3225_flange_z - min_th]) 
		rotate([0, 0, -hip_rotation()])
		servo_cap();
	// translate(hexhip_servo_offs)
	// 	translate([-10, 0, -hexhip_l + wire_clip_h + tab_th + min_th])
	// 	wire_clip();
}

!hexhip_design();

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
				translate([0, -25, -25])
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
		translate([-hexbody_pivot_r, 0, 0*(-openbeam_w - 2*hexrib_h)])
			rotate([180, 0, 30])
			hexframe();
		servo_cap();
	}
}

// ! hexrib();

center_or = 40;

module hexbody_center() {
	difference() {
		translate([0, 0, 0])
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
		hexframe();
	}
}
// ! hexbody_center();

servo_cap_h = ds3225_flange_z + 1.5;

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
			// ds3225(show_model=true);
			ds3225(show_model=false);
		}
		translate([0, 0, -ds3225_flange_z - m5_screw_cap_h])
			rotate([180, 0, 0])
			m5_screw();
			// rotate([0, 0, 30])
		// 	m5_nut();
		
		translate([0, 0, -ds3225_flange_l])
			cylinder(r=5/2, h=ds3225_flange_l*2, $fn=36);
		translate([10, 0, -21.5])
			cube([8, 8, 8], center=true);
	}

	% ds3225_horn_to_flange()
		ds3225(hip_rotation(), show_model=true);
}

// !servo_cap();

hexframe_l = 200;
hexframe_w = 100;
hexframe_r = 100;

module hexframe() {
	for (r=[0:60:hexbody_limit]) {
		rotate([0, 0, r]) {
			translate([hexframe_r/2 + openbeam_w, 0, openbeam_w/2 + min_th])
				rotate([0, 90, 0])
				openbeam(hexframe_r);
			translate([hexframe_r +3*openbeam_w/2, 0, openbeam_w/2 + min_th])
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
