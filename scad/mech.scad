include <hardware.scad>;

bracket_h = 2;
bracket_or = 50;
bracket_hole_or = 5.2/2;
bot_side = 300;

module corner_bracket(vert_slot=true) {
	// % for (r=[30, -30, -90])
	// 	translate([0, 0, bracket_h+10])
	// 	rotate([-90, 0, r])
	// 	translate([0, 0, bot_side/2])
	// 	t2020(bot_side);
	// % rotate([0, 0, -30])
	// 	translate([0, 6, bot_side/2])
	// 	t2020(bot_side);
	difference() {
		intersection() {
			cylinder(h=bracket_h, r=bracket_or, $fn=6);
			union() {
				translate([0, -10, 0])
					cube([bracket_or + 10, bracket_or + 10, bracket_h]);
				rotate([0, 0, 30])
					translate([-10, 0, 0])
					cube([bracket_or + 10, bracket_or + 10, bracket_h]);
				cylinder(r=10, h=bracket_h);
			}
		}
		for (r=[0, 60, 120]) {
			for (dx=[10, 20, 30, 50]) {
				rotate([0, 0, r])
				translate([bracket_or - dx, 0, 0])
				cylinder(r=bracket_hole_or, h=bracket_h, $fn=48);
			}
		}
		if (vert_slot) {
			rotate([0, 0, -30])
				translate([0, 2, 0])
				cube([20, 28, bracket_h*2+1], center=true);
		}
	}
	
}

module bottom_bracket() {
	corner_bracket(false);
}
// ! bottom_bracket();
// !corner_bracket();


center_or = 65;

module center_bracket() {
	difference() {
		cylinder(r=center_or, h=bracket_h, $fn=6);
		for (r=[0:60:360]) {
			for (dx=[10, 30]) {
				rotate([0, 0, r])
					translate([center_or - dx, 0, -1])
					cylinder(r=bracket_hole_or, h=bracket_h+2, $fn=48);
			}
		}
		rotate([0, 0, 30])
			cylinder(r=20, h=bracket_h+2, $fn=6);
	}
}
// ! center_bracket();

femur_angle = 80;

module femur_base_bracket() {
	difference() {
		translate([-10, -10, 0]) {
		
			intersection() {
				cube([40, 60, bracket_h]);
				translate([0, 20, 0])
					rotate([0, 0, femur_angle-90])
					translate([0, -20, 0])
					cube([50, 90, bracket_h]);
				translate([20, 20, 0])
					rotate([0, 0, 90-femur_angle])
					translate([-40, -30, 0])
					cube([60, 90, bracket_h]);
				* translate([40, 20, 0])
					rotate([0, 0, 15])
					translate([-40, 0, 0])
					cube([40, 70, bracket_h]);
				// cylinder(r=60, h=bracket_h);
			}
		}
		for (i=[-1, 1]) {
			translate([10 + i * 10, 0, 0]) {
				cylinder(r=bracket_hole_or, h=bracket_h, $fn=48);
				translate([0, 10, 0])
					rotate([0, 0, i*(90-femur_angle)]) 
					{
					for (y=[10:20:40])
						translate([0, y, 0])
							cylinder(r=bracket_hole_or, h=bracket_h, $fn=48);
				}
			}
		}
	}
	% translate([-10, 10, 0])
		rotate([0, 0, femur_angle-90])
		translate([10, 60, bracket_h+10])
		rotate([90, 0, 0])
		t2020(120);
	% //rotate([0, 0, femur_angle-90])
		translate([50, 0, bracket_h+10])
		rotate([90, 0, 90])
		t2020(120);

}

// femur_base_bracket();

knee_angle = 180 - 2 * femur_angle;
knee_top_w = 40;
knee_top_l = actuator_bracket_l;
module knee_top() {
	
	difference() {
		union() {
			translate([-10, -knee_top_w/2, 0])
				cube([knee_top_l, knee_top_w, bracket_h]);
			translate([10, knee_top_w/2, 0])
				rotate([0, 0, knee_angle/2])
				translate([0, -20, 0])
				cube([knee_top_l-20, 20, bracket_h]);
			translate([10, -knee_top_w/2, 0])
				rotate([0, 0, -knee_angle/2])
				cube([knee_top_l-20, 20, bracket_h]);
		}
		for (x=[0, actuator_bracket_hole_c_c])
			translate([x, 0, 0])
			cylinder(r=bracket_hole_or+0.5, h=bracket_h*2, $fn=48);
		for (i=[-1, 1]) {
			translate([0, i*10, 0]) {
				cylinder(r=bracket_hole_or, h=bracket_h*2, $fn=48);
				# translate([10, 0, 0])
					rotate([0, 0, i*knee_angle/2])
					for (x=[10:20:40])
					translate([x, 0, 0])
					cylinder(r=bracket_hole_or, h=bracket_h*2, $fn=48);
			}
		}
	}
	% translate([0, 0, bracket_h])
		actuator_bracket();
	% translate([10, -10, 0])
		rotate([0, 0, -knee_angle/2])
		translate([60, 0, -10])
		rotate([0, 90, 0])
		t2020(120);
	% translate([0, 0,-10])
		rotate([90, 0, 0])
		t2020(50);
	% translate([10, 10, 0])
		rotate([0, 0, knee_angle/2])
		translate([60, 0, -10])
		rotate([0, 90, 0])
		t2020(120);
}

// knee_top();

knee_bottom_w = 40;
knee_bottom_l = sk8_flange_w + 10;
knee_bottom_sk8_gap = 28;
module knee_bottom() {
	difference() {
		union() {
			translate([-10, -knee_bottom_w/2, 0])
				cube([knee_bottom_l, knee_bottom_w, bracket_h]);
			translate([10, knee_bottom_w/2, 0])
				rotate([0, 0, knee_angle/2])
				translate([0, -20, 0])
				cube([knee_bottom_l-20, 20, bracket_h]);
			translate([10, -knee_bottom_w/2, 0])
				rotate([0, 0, -knee_angle/2])
				cube([knee_bottom_l-20, 20, bracket_h]);
		}
		for (i=[-1, 1]) {
			// translate([0, i*10, 0]) {
			// 	// for (x=[0, sk8_hole_c_c])
			// 	// 	translate([x, 0, 0])
			// 	// 	cylinder(r=bracket_hole_or, h=bracket_h*2, $fn=48);
			// 	* cylinder(r=bracket_hole_or, h=bracket_h*2, $fn=48);
			// 	* translate([10, 0, 0])
			// 		rotate([0, 0, i*knee_angle/2])
			// 		for (x=[10:20:40])
			// 		translate([x, 0, 0])
			// 		cylinder(r=bracket_hole_or, h=bracket_h*2, $fn=48);
			// }
			# translate([0, i*knee_bottom_sk8_gap/2, 0]) {
				for (x=[0, sk8_hole_c_c])
					translate([x, 0, 0])
					cylinder(r=bracket_hole_or, h=100, center=true, $fn=48);
			}
		}

				
	}
	// % translate([0, 0, bracket_h])
	// 	actuator_bracket();
	% for (i=[-1, 1])
			// translate([0, , 0])
		translate([sk8_hole_c_c/2, i*knee_bottom_sk8_gap/2, sk8_c_h + bracket_h])
		rotate([0, 0, 90])
		sk8();
	% translate([10, -10, 0])
		rotate([0, 0, -knee_angle/2])
		translate([60, 0, -10])
		rotate([0, 90, 0])
		t2020(120);
	% translate([0, 0,-10])
		rotate([90, 0, 0])
		t2020(50);
	% translate([10, 10, 0])
		rotate([0, 0, knee_angle/2])
		translate([60, 0, -10])
		rotate([0, 90, 0])
		t2020(120);
}


knee_bottom();

FRAME_U = 300;
FRAME_DIAM = 635;

module hex_edge() {
	// FRAME_U beam cut at 30 deg
	difference() {
		t2020(FRAME_U);
		for (i=[-1, 1])
			translate([-t2020_w/2, -t2020_w/2, i*FRAME_U/2])
			rotate([-45 + i * 15, 0, 0])
			cube(2*[t2020_w, t2020_w, t2020_w]);
	}
}

// ! hex_edge();

module frame_layer() {
	for (i=[-1, 1]) {
		translate([0, 0, -bracket_h/2 + i * (bracket_h/2 + t2020_w/2)])
			center_bracket();	
	}
	rotate([0, 90, 0])
		t2020(FRAME_DIAM);
	for (r=[0:60:360]) {
		rotate([0, 0, r]) {
			translate([FRAME_DIAM/2 - FRAME_U/2, 0, 0])
				rotate([0, 90, 0])
				t2020(FRAME_U);
			translate([FRAME_DIAM/2, t2020_w/2, 0])
				rotate([0, 90, 120])
				translate([0, t2020_w/2, FRAME_U/2])
				hex_edge();
			translate([FRAME_DIAM/2 - t2020_w/3, 0, -t2020_w/2 - bracket_h])
				rotate([0, 0, 120])
				bottom_bracket();
			translate([FRAME_DIAM/2 - t2020_w/3, 0, t2020_w/2])
				rotate([0, 0, 120])
				corner_bracket();
		}
	}
}

module frame() {
	frame_layer();
	
	
}

// frame();

module spacer(id=5.1, od=7.9, h=15, clr=1) {
	difference() {
		union() {
			cylinder(r=od/2, h=h, $fn=48);
			translate([0, 0, h])
				sphere(r=od/2, $fn=48);
		}
		cylinder(r=id/2, h=3*h+1, $fn=48, center=true);
		translate([od/2, 0, 0])
			cube([od, clr, 3*h+1], center=true);
	}
}
// !spacer();


uj_clearance = 1;
uj_ir = actuator_nub_w * 1.5 / 2;

module universal_joint() {
	difference() {
		rotate([0, 90, 0])
			cylinder(r=actuator_nub_w, h=actuator_nub_w*2/3, center=true);
		cube([2*uj_ir, 2*uj_ir, actuator_nub_w - 2*uj_clearance], center=true);
		# cylinder(r=actuator_hole_ir, h=actuator_nub_w*2, center=true);
		# for (i=[-1, 1])
			translate([0, i*uj_ir, 0])
			rotate([-i*90, 0, 0]) {
				cylinder(r=1.5*actuator_hole_ir, h=actuator_hole_ir);
				cylinder(r=actuator_hole_ir, h=actuator_nub_w);
			}
	}
	// % actuator();

}

// ! universal_joint();

// leg_base_offs = 100;
// leg_foot_offs = 20;
// leg_foot_or = leg_foot_offs + actuator_nub_w;

// module leg_foot() {
// 	difference() {
// 		sphere(r=leg_foot_or);
// 		translate([0, 0, -leg_foot_or/2])
// 			cube([2*leg_foot_or, 2*leg_foot_or, leg_foot_or], center=true);
// 		for (i=[0:120:360])
// 			rotate([0, 0, i])
// 			translate([0, leg_foot_offs, 0])
// 			rotate([0, -90, 90]) {
// 				universal_joint();
// 			}
// 	}
// }

// function point_distance(p1, p2) = sqrt(
// 	(p2[0] - p1[0])^2 +
// 	(p2[1] - p1[1])^2 +
// 	(p2[2] - p1[2])^2);

// function point_diff(p1, p2) = [
// 	p2[0] - p1[0],
// 	p2[1] - p1[1],
// 	p2[2] - p1[2]];

// // rotation from p1 to p2
// function rot_difference(p1, p2) = [
// 	atan2(point_diff(p1, p2)[1], point_diff(p1, p2)[2]),
// 	atan2(point_diff(p1, p2)[0], point_diff(p1, p2)[2]),
// 	0,
// ];

// function rotate_point(p, rotation) = [
// 	/*
// 		rotate point p about the z axis.
// 		first convert (x, y) projection to polar,
// 		rotate polar,
// 		and reproject to (x', y')
// 	*/
// 	// r = point_distance([p[0], p[1], 0], [0, 0, 0])
// 	// theta = atan2(p[1], p[0])
// 	// p'[0] = r * sin(theta + rotation)
// 	// p'[1] = r * cos(theta + rotation)
// 	point_distance(
// 		[0, 0, 0], [p[0], p[1], 0]
// 		) * cos(atan2(p[1], p[0]) + rotation),
// 	point_distance(
// 		[0, 0, 0], [p[0], p[1], 0]
// 		) * sin(atan2(p[1], p[0]) + rotation),
// 	p[2]
// ];

// test_rot = 120;
// test_point = [0, leg_base_offs, 0];
// echo("rotate point(", test_point, ", ", test_rot, ") = ", rotate_point(test_point, test_rot));

// module leg_actuator(x=0, y=0, z=400, rotation=0) {
// 	l0_startpoint = rotate_point(
// 		[0, leg_base_offs, 0], rotation);
// 	l0_endpoint = [x, y, z] + rotate_point(
// 		[0, leg_foot_offs, 0], rotation);
// 	l0_rot = rot_difference(l0_startpoint, l0_endpoint);
// 	l0_ext = point_distance(l0_startpoint, l0_endpoint) - actuator_min_c_c;
// 	echo(
// 		l0_startpoint, 
// 		l0_endpoint, 
// 		actuator_min_c_c, 
// 		l0_ext, 
// 		l0_rot, 
// 		point_diff(l0_startpoint, l0_endpoint));
// 	rotate([0, 0, rotation])
// 		translate([0, leg_base_offs, 0]) {
// 			rotate(l0_rot) 
// 			rotate([0, -90, 0])
// 				# actuator(l0_ext);
// 			rotate([0, -90, 0])
// 				universal_joint();
// 		}
// }

// module leg(x=0, y=0, z=400) {
// 	for (rot=[0:120:180]) {
// 		leg_actuator(x, y, z, rot);
// 	}
	
// 	// rotate([0, 0, 360/3])
// 	// 	translate([0, leg_base_offs, 0])
// 	// 	rotate([0, 90, 0])
// 	// 	actuator(y);
// 	// rotate([0, 0, 2*360/3])
// 	// 	translate([0, leg_base_offs, 0])
// 	// 	rotate([0, 90, 0])
// 	// 	actuator(z);
// 	translate([x, y, z])
// 		leg_foot();
// }

// leg();