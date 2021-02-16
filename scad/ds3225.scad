
ds3225_l = 41;
ds3225_w = 20;
ds3225_h = 41.25;
ds3225_flange_l = 54.5;
ds3225_flange_z = 28;
ds3225_flange_h = 5;
ds3225_flange_hole_ir = 3.5/2;
ds3225_flange_holes_c_c = [49.5, 10];
ds3225_axle_flange_offset = 17.25;

ds3225_horn_l = 35.5;
ds3225_horn_w = 8;
ds3225_horn_h = 6.2;
ds3225_horn_d = 3.5;
ds3225_horn_hole_ir = 3/2;
ds3225_horn_or = 15/2;
ds3225_horn_hole_x = [0, 9, 20, 24];

ds3225_horn_dz = 1.0;

module ds3225_horn_holes(h=ds3225_horn_h*3, r=ds3225_horn_hole_ir) {
	for (x=ds3225_horn_hole_x) {
		translate([x, 0, 0])
			cylinder(r=r, h=h, $fn=24, center=true);
	}
}

module ds3225_horn() {
	difference() {
		union() {
			cylinder(r=ds3225_horn_or, h=ds3225_horn_h, $fn=36);
			translate([ds3225_horn_or/2-2, 0, 0])
				cylinder(r=ds3225_horn_or, h=ds3225_horn_h, $fn=36);
			translate([ds3225_horn_or-3, 0, 0])
				cylinder(r=ds3225_horn_or, h=ds3225_horn_h, $fn=36);
			translate([0, -ds3225_horn_w/2, 0])
				cube([
					ds3225_horn_l - ds3225_horn_or - ds3225_horn_w/2,
					ds3225_horn_w,
					ds3225_horn_h]);
			translate([ds3225_horn_l - ds3225_horn_or - ds3225_horn_w/2, 0, 0])
				cylinder(r=ds3225_horn_w/2, h=ds3225_horn_h, $fn=24);
		}
		ds3225_horn_holes();
	}
}

// ! ds3225_horn();

module ds3225_horn_stl() {
	import("ds3225_horn.stl");
}

module ds3225_center_to_horn() {
	translate([
		ds3225_axle_flange_offset-ds3225_flange_l/2, 
		0, -ds3225_h/2])
	children();
}

module ds3225(rotation=0, show_model=false) {
	ds3225_center_to_horn() {
		if (show_model) {
			% import("DS3225.stl");
		} else {
			cube([ds3225_l, ds3225_w, ds3225_h], center=true);
		}
	}
	translate([0, 0, ds3225_horn_dz]) 
		rotate([0, 0, rotation]) {
		if (show_model) {
			ds3225_horn();
		} else {
			ds3225_horn_stl();
		}
		children();
	}
}

// !ds3225();
module ds3225_horn_to_flange(dir=1) {
	translate(dir * [0, 0, ds3225_h - ds3225_flange_z]) 
	children();
}

module ds3225_holes(r=ds3225_flange_hole_ir, h=2*ds3225_h) {
	ds3225_center_to_horn() {
		for (x=[-1, 1]) {
			for (y=[-1, 1]) {
				translate([
					x * ds3225_flange_holes_c_c[0]/2,
					y * ds3225_flange_holes_c_c[1]/2,
					0])
				cylinder(r=r, h=h, $fn=36, center=true);
			}
		}
		// cube([ds3225_l + 1, ds3225_w + 1, ds3225_h], center=true);
	}
}

// !ds3225();