include <ds3225.scad>;
// include <openbeam.scad>;
include <t20beam.scad>;
include <metric.scad>;

// function hip_rotation() = hip_rom * abs(sin(180*$t)) - hip_rom/2;
// function knee_rotation() = -knee_rom * abs(sin(180*$t*2)) + knee_rom/2 + knee_bias;
// function foot_rotation() = foot_rom * abs(sin(180*$t*4)) - foot_rom/2;

ankle_bias = -45;

test_a = 90;

hip_angle = test_a/2;
knee_angle = -test_a/3;
ankle_angle = test_a;

function hip_rotation() = hip_angle;
function knee_rotation() = knee_angle;
function ankle_rotation() = ankle_bias + ankle_angle;


max_th = 4;
// min_th = 2;
fork_w=51;
fork_d=50;

module motor_plate() {
	difference() {
		union() {
			translate([-ds3225_flange_l + ds3225_axle_flange_offset, -ds3225_w/2 - max_th, -max_th])
				ds3225_horn_to_flange(-1)
				cube([
					ds3225_flange_l, ds3225_w + t20_w + max_th, max_th]);
			* translate([
					-ds3225_flange_l + ds3225_axle_flange_offset, 
					-ds3225_w/2 - max_th, 
					-(max_th + t20_w/2)])
				ds3225_horn_to_flange(-1)
				cube([
					ds3225_flange_l, max_th, t20_w/2]);
		
		}
		ds3225();
		ds3225_holes();
		for (dx=[-ds3225_axle_flange_offset/2:t20_w:ds3225_flange_l - 10])
			translate([-dx, ds3225_w/2 + t20_w/2, 0])
			cylinder(r=5.2/2, h=50, center=true, $fn=32);
	}
}

// ! motor_plate();

module bottom_plate() {
	difference() {
		translate([0, 0, -t20_w - max_th])
		union() {
			translate([
					-ds3225_flange_l + ds3225_axle_flange_offset, 
					-ds3225_w/2 - max_th, 
					-2*max_th])
				ds3225_horn_to_flange(-1)
				cube([
					ds3225_flange_l, 
					ds3225_w + t20_w + max_th, 
					2*max_th]);
			* translate([
					-ds3225_flange_l + ds3225_axle_flange_offset, 
					-ds3225_w/2 - max_th, 
					0])
				ds3225_horn_to_flange(-1)
				cube([
					ds3225_flange_l, max_th, t20_w/2]);
			* translate([-ds3225_flange_l + ds3225_axle_flange_offset, -ds3225_w/2 - max_th, -2*max_th])
				ds3225_horn_to_flange(-1)
				cube([
					ds3225_flange_l, ds3225_w + 2*max_th, max_th]);
			
		}
		ds3225();
		// % ds3225(show_model=true);
		ds3225_holes();
		translate([0, 0, -23.2])
			ds3225_holes(r=5, h=3);
		# translate([0, 0, -43.7])
			rotate([180, 0, 0])
			m5_screw();
		for (dx=[-ds3225_axle_flange_offset/2:t20_w:ds3225_flange_l - 10]) {
			translate([-dx, ds3225_w/2 + t20_w/2, -42])
				// rotate([180, 0, 0])
				m5_screw(10);
			// translate([-dx, ds3225_w/2 + t20_w/2, -30])
			// 	cylinder(r=5.2/2, h=50, center=true, $fn=32);
			translate([-dx, ds3225_w/2 + t20_w/2, -43.3])
				cylinder(r=5, h=max_th, center=true, $fn=32);
	}	}
}

// ! bottom_plate();

module pivot_plate() {
	pivot_plate_l = 40;
	difference() {
		union() {
			translate([-t20_w/2, -t20_w/2, 0])
				cube([pivot_plate_l, t20_w, max_th]);
			cylinder(r=5, h=4.5);
		}
		cylinder(r=5.2/2, h=100, center=true);
		for (dx=[10:10:pivot_plate_l-20])
			translate([dx, 0, 1.5])
			rotate([180, 0, 0])
			# m5_screw();
			* cylinder(r=5.2/2, h=100, center=true);
	}
}

module h_bracket() {
	difference() {
		union() {
			translate([fork_d - t20_w, 0, 0])
				cube([2 * t20_w, max_th, t20_w]);
			translate([fork_d - t20_w, 0, -fork_w - t20_w])
				cube([2 * t20_w, max_th, t20_w]);
			translate([fork_d, 0, -fork_w])
				cube([t20_w, max_th, fork_w]);
		}
		for (dx=[0, t20_w]) {
			for (dz=[0:(fork_w + t20_w)/3:fork_w + t20_w]) {
				translate([t20_w/2 + fork_d - dx, 0, t20_w/2 - dz])
					rotate([90, 0, 0])
					cylinder(r=5.2/2, h=40, center=true);
			}
		}
	}
}

module joint_frame(horn_beam_l=20) {
	% translate([horn_beam_l/2 + 9, 0, t20_w/2])
		rotate([0, 90, 0])
		t20(horn_beam_l);
	% translate([fork_d + 9, 0, -fork_w/2])
		rotate([0, 0, 0])
		t20(fork_w);
	% translate([fork_d - t20_w , 0, -t20_w/2 - fork_w])
		rotate([0, 90, 0])
		t20(fork_d + 3/2*t20_w);
	translate([0, t20_w/2, 0])
		h_bracket();
	translate([0, -t20_w/2 - max_th, 0])
		h_bracket();
}

// !joint_frame();

module motor_plate_design(rotation=90, horn_beam_l=60, motor_beam_l=50) {
	motor_plate();
	bottom_plate();
	% translate([-motor_beam_l/2 + ds3225_axle_flange_offset, ds3225_w/2 + t20_w/2, -t20_w/2 - max_th])
		ds3225_horn_to_flange(-1)
		rotate([0, 90, 0])
		t20(motor_beam_l);
	ds3225(rotation, show_model=true) {
		joint_frame(horn_beam_l);
		translate([0, 0, -fork_w])
			pivot_plate();
	
	}
	// #translate([0, 0, ds3225_horn_h - 52.5 + ds3225_horn_dz])
	// 	cube([5, 50, 52.5]);
}

// ! motor_plate_design();

module leg_design() {
	// 
	translate([
		-150 + 8, 
		t20_w, 
		-max_th])
		ds3225_horn_to_flange(-1)
		rotate([180, 0, ankle_rotation()]) {
			motor_plate_design(ankle_rotation(), 150, 150);
			// ds3225(hip_rotation(), show_model=true);
		}
}

// !leg_design();

module hip_frame(horn_beam_l=20) {
	% translate([horn_beam_l/2 + 9, 0, t20_w/2])
		rotate([0, 90, 0])
		t20(horn_beam_l);
	% translate([
			horn_beam_l + max_th/2, 
			0, 
			-t20_w/2 - 2*fork_w/3 - max_th/2])
		rotate([0, 0, 0])
		t20(fork_w);
	% translate([
			fork_d - t20_w + max_th/2, 
			0, -t20_w/2 - fork_w])
		rotate([0, 90, 0])
		t20(fork_d + 3/2*t20_w);
	* translate([0, t20_w/2, 0])
		h_bracket(fork_w, fork_d);
	* translate([0, -t20_w/2 - max_th, 0])
		h_bracket(fork_w, fork_d);
}

// !hip_frame();


module hip_motor_plate() {
	difference() {
		union() {
			translate([
					-ds3225_flange_l + ds3225_axle_flange_offset, 
					-ds3225_w/2, -max_th])
				ds3225_horn_to_flange(-1) {
					translate([0, -fork_w, 0])
						cube([
							ds3225_flange_l, 
							// ds3225_w + t20_w + max_th, 
							fork_w + t20_w + ds3225_w,
							max_th]);
					// translate([0, -max_th, 0])
					// 	cube([
					// 	ds3225_flange_l,
					// 	ds3225_w + t20_w + max_th, 
					// 	max_th]);
					// translate([
					// 		ds3225_flange_l - t20_w, 
					// 		- fork_w, 0])
					// 	cube([t20_w, fork_w, max_th]);
					// translate([
					// 		0, 
					// 		0*ds3225_w/2 - fork_w, 0])
					// 	cube([ds3225_flange_l - t20_w, 
					// 		t20_w, max_th]);
				// % translate([0, 0, 0])
				// 	cube([t20_w, 2 * t20_w, max_th]);
				// % translate([0, 0, 0])
				// 	cube([t20_w, fork_w, max_th]);
				}
			* translate([
					-ds3225_flange_l + ds3225_axle_flange_offset, 
					-ds3225_w/2 - max_th, 
					-(max_th + t20_w/2)])
				ds3225_horn_to_flange(-1)
				cube([
					ds3225_flange_l, max_th, t20_w/2]);
			
			
		}
		for (dx=[7:-t20_w:-fork_d+2]) {
			for (dy=[t20_w, -fork_w, -fork_w + t20_w]) {
				translate([dx, dy, 0])
					// rotate([90, 0, 0])
					cylinder(r=5.2/2, h=40, center=true);
			}
		}
	
		
		ds3225();
		ds3225_holes();
		* for (dx=[-ds3225_axle_flange_offset/2:t20_w:ds3225_flange_l - 10])
			translate([-dx, ds3225_w/2 + t20_w/2, 0])
			cylinder(r=5.2/2, h=50, center=true, $fn=32);
	}
}

// ! hip_motor_plate();

module hip_bottom_plate() {
	difference() {
		translate([0, 0, -t20_w - max_th])
		union() {
			translate([
					-ds3225_flange_l + ds3225_axle_flange_offset, 
					-ds3225_w/2 - max_th, 
					-2*max_th])
				ds3225_horn_to_flange(-1) {
					translate([0, -fork_w + max_th, 0])
						cube([
							ds3225_flange_l, 
							// ds3225_w + t20_w + max_th, 
							fork_w + t20_w + ds3225_w,
							2*max_th]);
					// translate([
					// 		ds3225_flange_l - t20_w, 
					// 		-fork_w + max_th, 0])
					// 	cube([t20_w, fork_w, 2*max_th]);
					// translate([
					// 		0, 
					// 		-fork_w + max_th, 0])
					// 	cube([ds3225_flange_l - t20_w, 
					// 		t20_w, 2*max_th]);
			}
		
			* translate([
					-ds3225_flange_l + ds3225_axle_flange_offset, 
					-ds3225_w/2 - max_th, 
					0])
				ds3225_horn_to_flange(-1)
				cube([
					ds3225_flange_l, max_th, t20_w/2]);
			* translate([-ds3225_flange_l + ds3225_axle_flange_offset, -ds3225_w/2 - max_th, -2*max_th])
				ds3225_horn_to_flange(-1)
				cube([
					ds3225_flange_l, ds3225_w + 2*max_th, max_th]);
			
		}
		ds3225();
		// % ds3225(show_model=true);
		ds3225_holes();
		translate([0, 0, -23.2])
			ds3225_holes(r=5, h=3);
		# translate([0, 0, -43.7])
			rotate([180, 0, 0])
			m5_screw();
		// for (dx=[-ds3225_axle_flange_offset/2:t20_w:ds3225_flange_l - 10]) {
		// 	translate([-dx, ds3225_w/2 + t20_w/2, -42])
		// 		// rotate([180, 0, 0])
		// 		m5_screw(10);
		// 	// translate([-dx, ds3225_w/2 + t20_w/2, -30])
		// 	// 	cylinder(r=5.2/2, h=50, center=true, $fn=32);
		// 	translate([-dx, ds3225_w/2 + t20_w/2, -43.3])
		// 		cylinder(r=5, h=max_th, center=true, $fn=32);
		// }	
		for (dx=[7:-t20_w:-fork_d]) {
			for (dy=[t20_w, -fork_w, -fork_w + t20_w]) {
				translate([dx, dy, -42])
					// rotate([180, 0, 0])
					 m5_screw(10);
				translate([dx, dy, -44])
					cylinder(r=5, h=max_th, center=true, $fn=32);
		
					// cylinder(r=5.2/2, h=40, center=true);
			}
		}
	}
}

// ! hip_bottom_plate();

module knee_plate_design(rotation=90, horn_beam_l=80, motor_beam_l=50, fork_w=51, fork_d=50) {
	motor_plate();
	bottom_plate();
	% translate([-motor_beam_l/2 + ds3225_axle_flange_offset, ds3225_w/2 + t20_w/2, -t20_w/2 - max_th])
		ds3225_horn_to_flange(-1)
		rotate([0, 90, 0])
		t20(motor_beam_l);
	ds3225(-rotation, show_model=true) {
		hip_frame(horn_beam_l);
		translate([0, 0, -fork_w])
			pivot_plate();
	
	}
	// #translate([0, 0, ds3225_horn_h - 52.5 + ds3225_horn_dz])
	// 	cube([5, 50, 52.5]);
}

module hip_plate_design(rotation=90, horn_beam_l=60, motor_beam_l=50) {
	hip_motor_plate();
	hip_bottom_plate();
	% translate([-motor_beam_l/2 + ds3225_axle_flange_offset, ds3225_w/2 + t20_w/2, -t20_w/2 - max_th])
		ds3225_horn_to_flange(-1)
		rotate([0, 90, 0])
		t20(motor_beam_l);
	ds3225(rotation, show_model=true) {
		joint_frame(horn_beam_l);
		translate([0, 0, -fork_w])
			pivot_plate();
	
	}
	// #translate([0, 0, ds3225_horn_h - 52.5 + ds3225_horn_dz])
	// 	cube([5, 50, 52.5]);
}

// ! hip_plate_design();

module hip_design() {
	hip_plate_design(hip_rotation(), 150, 68);
	// * translate([-15, ds3225_w/2 + t20_w/2, -t20_w/2 - max_th])
	// 	ds3225_horn_to_flange(-1)
	// 	rotate([0, 90, 0])
	// 	t20(70);
	// % ds3225(hip_rotation(), show_model=true) {
	// 	translate([25 + 9, 0, t20_w/2])
	// 	rotate([0, 90, 0])
	// 	t20(50);
	// }
	translate([-75, ds3225_w/2 - ds3225_horn_dz, -max_th - t20_w/2])
		ds3225_horn_to_flange(-1)
		rotate([-90, knee_rotation(), 0]) {
			leg_design();
			
			//TODO: fix
			knee_plate_design(knee_rotation());
			// ds3225(hip_rotation(), show_model=true);
		}
}

! hip_design();

module body_design() {
	for (r=[0:60:300]) {
		rotate([0, 0, r])
		translate([-180, 0, 0])
			rotate([0, 0, -hip_rotation()])
			hip_design();
	}
}

body_design();
