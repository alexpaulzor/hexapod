include <ds3225.scad>;


// sj_th = 2;
// sj_w = ds3225_w + 2*sj_th;
// sj_l = ds3225_l + 2 * sj_th;
// sj_h = ds3225_flange_z + 5;
// sj_axle_h = 5;
// sj_axle_ir = 5.5/2;
// sj_axle_or = 5;
// module servo_jacket() {
// 	// adds coaxial pivot for bottom of ds3225 servo
// 	difference() {
// 		union() {
// 			translate([
// 					ds3225_axle_flange_offset-ds3225_flange_l/2, 
// 					0, -sj_h/2]) 
// 				cube([sj_l, sj_w, sj_h], center=true);
// 			* translate([0, 0, -sj_h - sj_axle_h])
// 				cylinder(r1=sj_axle_or, r2=2*sj_axle_or, h=sj_axle_h);
// 			translate([
// 					ds3225_axle_flange_offset-ds3225_flange_l/2, 
// 					0, -sj_th/2])
// 				cube([ds3225_flange_l, sj_w, sj_th], center=true);
// 			translate([
// 					ds3225_axle_flange_offset - 2, 
// 					-sj_w/2, -sj_th - 0.5])
// 				rotate([0, 225, 0])
// 				cube([3, sj_w, 10]);
// 			translate([
// 					ds3225_axle_flange_offset, 
// 					-sj_w/2, -sj_th])
// 				rotate([0, 225, 0])
// 				cube([2, sj_w, 10]);

// 			translate([
// 					ds3225_axle_flange_offset - ds3225_flange_l + 2, 
// 					-sj_w/2, -sj_th - 0.5])
// 				rotate([0, 45, 0])
// 				cube([10, sj_w, 3]);
// 			translate([
// 					ds3225_axle_flange_offset - ds3225_flange_l, 
// 					-sj_w/2, -sj_th])
// 				rotate([0, 45, 0])
// 				cube([10, sj_w, 2]);
// 		}
// 		translate([
// 				ds3225_axle_flange_offset-ds3225_flange_l/2, 
// 				0, -sj_h/2])
// 			translate([20, 0, 0])
// 				cube([10, 8, 16], center=true);
// 		cylinder(r=sj_axle_ir, h=3*sj_h, center=true, $fn=36);
// 		ds3225_horn_to_flange() {
// 			ds3225();
// 			ds3225_holes();
// 		}
// 	}

// }

// ! servo_jacket();


// $t = 0.4;
// $t = 0;
axle_or = 5.2/2;

hip_rom = 120;
knee_bias = -90;
knee_rom = 180;
foot_rom = 180;

function hip_rotation() = hip_rom * abs(sin(180*$t)) - hip_rom/2;
function knee_rotation() = -knee_rom * abs(sin(180*$t*2)) + knee_rom/2 + knee_bias;
function foot_rotation() = foot_rom * abs(sin(180*$t*4)) - foot_rom/2;

// hexfoot_l = 100;
// hexfoot_h = 8;
// hexfoot_w = 15;
// hexleg_d = 24;
hexfoot_w = 50;
hexfoot_l = 100;
hexfoot_h = ds3225_horn_h + 2;
hexfoot_d = 20;

hexfoot_bridge_l = 45;

module hexfoot() {
	// // ds3225_horn()
	// * difference() {
	// 	translate([-ds3225_horn_or, -hexfoot_w/2, 0])
	// 		cube([hexfoot_l + ds3225_horn_or, hexfoot_w, hexfoot_h]);
	// 	*translate([-ds3225_horn_or, -hexfoot_w/2, -hexfoot_h + ds3225_horn_h + 2])
	// 		cube([hexfoot_l + ds3225_horn_or, hexfoot_w, hexfoot_h]);
	// 	ds3225_horn();
	// 	* % ds3225();
	// }
	// * translate([hexfoot_l/2, 0, -hexfoot_w/2])
	// 	scale([hexfoot_l/hexfoot_w, 1/2, 1])
	// 	sphere(r=hexfoot_w/2);
	difference() {
		union() { 
			translate([0, -hexfoot_d/2, 0])
				cube([hexfoot_bridge_l, hexfoot_d, hexfoot_h]);
			translate([0, -hexfoot_d/2, -48])
				cube([hexfoot_bridge_l, hexfoot_d, hexbody_th]);
			translate([0, 0, 6.2])
				cylinder(r=hexfoot_d/2, h=hexbody_th, $fn=36);
			translate([0, 0, -48])
				cylinder(r=hexfoot_d/2, h=hexbody_th, $fn=36);
			translate([hexfoot_bridge_l, -hexfoot_d/2, -hexfoot_w + hexbody_th])
				cube([3, hexfoot_d, hexfoot_w + hexfoot_h - hexbody_th]);
			* translate([hexfoot_l, 0, -2])
				ds3225_center_to_horn()
				cube([ds3225_flange_l, hexfoot_d, 20], center=true);
			translate([hexfoot_bridge_l + (hexfoot_l - hexfoot_bridge_l)/2, 0, -hexfoot_w/2 + 2.4])
				rotate([0, 90, 0])
				translate([-ds3225_horn_h/2 + 0.4, 0, 0])
				intersection() {
					cylinder(r1=(hexfoot_w+hexfoot_h)/2+4, r2=0, h=hexfoot_l - hexfoot_bridge_l, center=true);
					cube([(hexfoot_w + ds3225_horn_h), hexfoot_d, 100], center=true);
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
		translate([0, 0, -hexhip_riser_h])
			cylinder(r=5/2, h=hexhip_riser_h*2, $fn=36);
		* translate([hexfoot_l, 0, 0])
			rotate([0, 0, 0]) {
				ds3225(show_model=false);
				ds3225_holes();
				translate([0, 10, -2])
					ds3225_center_to_horn()
					cube([ds3225_l-10, 10, 20+1], center=true);
				translate([0, 0, -hexhip_riser_h])
					cylinder(r=axle_or, h=hexhip_riser_h*2, $fn=36);
			}
		translate([hexfoot_bridge_l + (hexfoot_l - hexfoot_bridge_l)/2, 0, -hexfoot_w/2 + 2.4])
				rotate([0, 90, 0])
				translate([-ds3225_horn_h/2 + 0.4, 0, 0])
					cylinder(r1=(hexfoot_w+hexfoot_h)/2 - hexbody_th*2, r2=0, h=(hexfoot_l - hexfoot_bridge_l)-6, center=true);
		// translate([0, 0, -50])
		// 	cube([100, 100, 100]);
	}
}

// !hexfoot();

hexleg_w = 50;
hexleg_l = 100;
hexleg_h = ds3225_horn_h + 2;
hexleg_d = 24;

hexleg_bridge_l = 45;

module hexleg() {
	difference() {
		union() { 
			translate([0, -hexleg_d/2, 0])
				cube([hexleg_bridge_l, hexleg_d, hexleg_h]);
			translate([0, -hexleg_d/2, -48])
				cube([hexleg_bridge_l, hexleg_d, 3]);
			translate([0, 0, 6.2])
				cylinder(r=hexleg_d/2, h=2, $fn=36);
			translate([0, 0, -48])
				cylinder(r=hexleg_d/2, h=3, $fn=36);
			translate([hexleg_bridge_l, -hexleg_d/2, -hexleg_w + hexbody_th])
				cube([3, hexleg_d, hexleg_w + hexleg_h - hexbody_th]);
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
		translate([0, 0, -hexhip_riser_h])
			cylinder(r=5/2, h=hexhip_riser_h*2, $fn=36);
		translate([hexleg_l, 0, 0])
			rotate([0, 0, 0]) {
				ds3225(show_model=false);
				ds3225_holes();
				translate([0, 10, -2])
					ds3225_center_to_horn()
					cube([ds3225_l-10, 10, 20+1], center=true);
				translate([0, 0, -hexhip_riser_h])
					cylinder(r=axle_or, h=hexhip_riser_h*2, $fn=36);
			}
		translate([hexleg_bridge_l + 10, 0, -hexleg_w/2 + 2.4])
			rotate([90, 0, 0])
			cylinder(r=8, h=26, center=true);
	}
	// translate([hexleg_l, 0, 0])
	// rotate([0, 0, 0])
	// % ds3225(knee_rotation());
}
// ! hexleg();

module hexleg_design() {
	hexleg();
	% ds3225_horn();
	translate([hexleg_l, 0, 0])
		rotate([0, 0, 0])
		ds3225(foot_rotation(), show_model=true) {
			hexfoot();
		}
}

// ! hexleg_design();

hexhip_l = 47;
hexhip_w = 20;
hexhip_h = 8;
hexhip_th = 2;
hexhip_riser_h = ds3225_flange_l;
hexhip_riser_w = ds3225_w + hexhip_th * 2;
hexhip_riser_clearance = hexhip_th * 2;

module hexhip() {
	difference() {
		union() {
			translate([0, -hexhip_w/2, 0]) {
				cube([hexhip_l, hexhip_w, hexhip_h]);
			}
			translate([0, 0, hexhip_h - hexhip_th])
				cylinder(r=hexhip_w/2, h=hexhip_th);
			translate([
					hexhip_l - hexhip_riser_w, 
					-hexhip_w/2, 
					-ds3225_flange_l + hexhip_h - hexhip_th])
				cube([
					hexhip_riser_w, 
					hexhip_w, 
					hexhip_riser_h]);

			translate([0, -hexhip_w/2, -hexhip_riser_h + hexhip_h - hexhip_th]) {
				cube([hexhip_l, hexhip_w, 3]);
			}
			translate([0, 0, -hexhip_riser_h + hexhip_h - hexhip_th]) 
				cylinder(r=hexhip_w/2, h=3);
			translate([hexhip_l - 20, 0, -ds3225_axle_flange_offset - 3])
				cube([20, 22, 20]);
		}
		translate([0, 0, -hexhip_riser_h])
			cylinder(r=axle_or, h=hexhip_riser_h*2, $fn=36);
		translate([0, -hexhip_w/2, -hexhip_riser_h + hexhip_h + hexhip_th])
			cube([hexhip_l - hexhip_riser_w, hexhip_w, hexhip_riser_clearance]);
		// # translate([-5, 0, 0])
		// 	ds3225_horn(); 
		ds3225_horn();
		translate([0, 0, hexhip_h-2])
			ds3225_horn_holes();
		translate([0, 0, 3])
			ds3225_horn_holes();
		translate([35, -hexleg_w/2, -10])
			rotate([90, -90, 0])
			ds3225_holes();
		translate([35, -23, -10])
			rotate([90, -90, 0]) {
				ds3225(knee_rotation(), show_model=false);
				translate([0, 0, -hexhip_riser_h])
					cylinder(r=axle_or, h=hexhip_riser_h*2, $fn=36);
			}

		translate([25, 0, -ds3225_l/2])
			cube([10, hexhip_w+1, ds3225_l-10], center=true);
	}
}

// ! hexhip();

module hexhip_design() {
	hexhip();
	ds3225_horn();
	translate([35, -23, -10])
		rotate([90, -90, 0])
		ds3225(knee_rotation(), show_model=true) 
		hexleg_design();
}

// !hexhip_design();

hexbody_or = 125;
hexbody_th = 2;
hexbody_pivot_r = 100;
hexbody_limit = 360;
hexbody_riser_l = 20;
hexbody_riser_w = 24;
hexbody_riser_h = ds3225_flange_z;

module hexbody_wafer(include_ds3225=true) {
	difference() {
		union() {
			rotate([0, 0, 0])
				cylinder(r=hexbody_or, h=hexbody_th, $fn=6);
			// # translate([0, 0, hexbody_riser_h])
			// 	hexbody_wafer();
			// // *for (r=[0:60:hexbody_limit]) {
			// // 	rotate([0, 0, r])
			// // 	translate([hexbody_pivot_r - hexbody_riser_w + ds3225_horn_or, -hexbody_riser_w/2, hexbody_th])
			// // 	# cube([hexbody_riser_l, hexbody_riser_w, hexbody_riser_h]);
			// // }
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
					translate([0, 0, -hexhip_riser_h])
						cylinder(r=axle_or, h=hexhip_riser_h*2, $fn=36);
				}
				translate([ds3225_axle_flange_offset + 2, -10, -2*hexbody_th])
					cube([10, 20, 4*hexbody_th]);
			}
			rotate([0, 0, 30+r]) {
				translate([hexbody_or + 2, 0, -2*hexbody_th])
				cylinder(r=hexbody_pivot_r/2, h=4*hexbody_th);
				# for (x=[15:15:60]) {
					translate([x, 0, -1])
					cylinder(r=3/2, h=hexbody_th*2, $fn=36);
				}
				cylinder(r=3/2, h=hexbody_th*2, $fn=36);
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
module hexbody_top() {
	translate([0, 0, -hexbody_th/2])
		hexbody_wafer(true);
	translate([0, 0, hexbody_th/2])
		hexbody_wafer(false);
}
// ! hexbody_top();

module servo_cap() {
	// % ds3225(show_model=true);
	corner_r = (ds3225_flange_l - ds3225_l) / 2 - hexbody_th;
	difference() {
		ds3225_center_to_horn()
		ds3225_horn_to_flange() {
			translate([0, 0, -hexbody_th -(ds3225_flange_z + hexbody_th)/4])
				cube([ds3225_l + 2 * hexbody_th, 
					20, ds3225_flange_z + hexbody_th], center=true);
			translate([0, 0, 4.5])
				cube([ds3225_flange_l, 
					20-2*corner_r, hexbody_th], center=true);
			for (x=[-1, 1]) {
				for (y=[-1, 1]) {
					translate([x*(ds3225_flange_l/2 - corner_r), y*(10-corner_r), 4.5])
						cylinder(r=corner_r, h=hexbody_th, center=true);
				}
			}
		}
		ds3225_horn_to_flange() {
			ds3225_holes();
			ds3225(show_model=false);
		}
		translate([0, 0, -hexhip_riser_h])
			cylinder(r=5/2, h=hexhip_riser_h*2, $fn=36);
		translate([10, 0, -21.5])
			cube([8, 8, 8], center=true);
	}

	* ds3225_horn_to_flange()
		ds3225(show_model=true);
}

// !servo_cap();

module hexbody() {
	// difference() {
	// 	union() {
	// 		rotate([0, 0, 0])
	// 			hexbody_wafer();
	// 		# translate([0, 0, hexbody_riser_h])
	// 			hexbody_wafer();
	// 		// *for (r=[0:60:hexbody_limit]) {
	// 		// 	rotate([0, 0, r])
	// 		// 	translate([hexbody_pivot_r - hexbody_riser_w + ds3225_horn_or, -hexbody_riser_w/2, hexbody_th])
	// 		// 	# cube([hexbody_riser_l, hexbody_riser_w, hexbody_riser_h]);
	// 		// }
	// 	}
	// 	for (r=[0:60:hexbody_limit]) {
	// 		rotate([0, 0, r])
	// 		translate([hexbody_pivot_r, 0, 0])
	// 		rotate([180, 0, 0])
	// 		ds3225_horn_to_flange() {
	// 			ds3225(hip_rotation())
	// 				# hexhip();
	// 			ds3225_holes();
	// 		}

	// 	}
	// }
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

// ! ds3225(hip_rotation())
// 	hexleg();
hexbody();

// */