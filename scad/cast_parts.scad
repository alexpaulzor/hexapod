include <casting.scad>;
include <ds3225.scad>;

module cast_ds3225_horn_holes(h=ds3225_horn_h+1, r=ds3225_horn_hole_ir) {
	for (x=ds3225_horn_hole_x) {
		translate([x, 0, ds3225_horn_h/2])
			draft_cylinder(r=r, h=h, $fn=24, center=true);
	}
}

module cast_ds3225_horn() {
	difference() {
		union() {
			draft_cylinder(r=ds3225_horn_or, h=ds3225_horn_h, draft_angle=-draft_angle, invert=true, $fn=36);
			translate([ds3225_horn_or/2-2, 0, 0])
				draft_cylinder(r=ds3225_horn_or, h=ds3225_horn_h, draft_angle=-draft_angle, invert=true, $fn=36);
			translate([ds3225_horn_or-3, 0, 0])
				draft_cylinder(r=ds3225_horn_or, h=ds3225_horn_h, draft_angle=-draft_angle, invert=true, $fn=36);
			translate([0, -ds3225_horn_w/2, 0])
				draft_cube([
					ds3225_horn_l - ds3225_horn_or - ds3225_horn_w/2,
					ds3225_horn_w,
					ds3225_horn_h], draft_angle=-draft_angle, invert=true);
			translate([ds3225_horn_l - ds3225_horn_or - ds3225_horn_w/2, 0, 0])
				draft_cylinder(r=ds3225_horn_w/2, h=ds3225_horn_h, draft_angle=-draft_angle, invert=true, $fn=24);
		}
		cast_ds3225_horn_holes();
	}
	// %ds3225_horn();
}

// ! cast_ds3225_horn();