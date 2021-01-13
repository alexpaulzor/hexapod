t2020_w = 20;
t2020_notch_w = 6;
t2020_notch_d = 6;
t2020_hole_ir = 5/2;

module t2020(length=100) {
    translate([0, 0, -length/2])
    linear_extrude(length) {
        difference() {
            square([t2020_w, t2020_w], center=true);
            for (i=[0:3]) {
                rotate([0, 0, 90 * i]) {
                    translate([t2020_w/2 - t2020_notch_d / 2, 0, 0]) {
                        square([t2020_notch_d, t2020_notch_w], center=true);
                    }
                }
            }
            circle(r=t2020_hole_ir);
        }
    }
}

module t2020_hole(length) {
    t2020(length);
    cube([t2020_w - 2, t2020_w - 2, length+1], center=true);
}


rail_r = 8 / 2;

sk8_hole_r = 5 / 2;
sk8_hole_c_c = 32;
sk8_c_h = 20;

sk8_flange_w = 42;
sk8_flange_h = 6.5;
sk8_w = 20;
sk8_l = 14;
sk8_h = 33.5;

module rail(l=400, center=false) {
    color("lightgrey") 
    rotate([0, 90, 0])
        cylinder(r=rail_r, h=l, center=center);
}

module sk8() {
    difference() {
        union() {
            translate([-sk8_l / 2, -sk8_w / 2, -sk8_c_h])
                cube([sk8_l, sk8_w, sk8_h]);
            translate([-sk8_l / 2, -sk8_flange_w / 2, -sk8_c_h]) 
                cube([sk8_l, sk8_flange_w, sk8_flange_h]);
        }
        translate([-sk8_l / 2, 0, 0])
            rail(sk8_l);
        sk8_holes();
    }
}

module sk8_holes(h=sk8_c_h) {
    translate([0, sk8_hole_c_c / 2, -sk8_c_h])
        cylinder(r=sk8_hole_r, h=h);
    translate([0, -sk8_hole_c_c / 2, -sk8_c_h])
        cylinder(r=sk8_hole_r, h=h);
}

actuator_motor_h = 98;
actuator_motor_or = 15;
actuator_lower_or = 15;
actuator_lower_h = 350;  // TODO: measure
actuator_nub_w = 15;
actuator_hole_ir = 3;
actuator_upper_or = 10;
actuator_min_c_c = 370;
actuator_max_c_c = 620;
actuator_stroke = actuator_max_c_c - actuator_min_c_c;

module actuator(pos=0) {
    difference() {
        cube([actuator_nub_w, actuator_nub_w, actuator_nub_w], center=true);
        cylinder(r=actuator_hole_ir, h=actuator_nub_w, center=true);
    }
    translate([actuator_nub_w/2, 0, 0])
        rotate([0, 90, 0])
        cylinder(r=actuator_lower_or, h=actuator_lower_h);
    translate([actuator_nub_w/2, actuator_lower_or + actuator_motor_or, 0])
        rotate([0, 90, 0])
        cylinder(r=actuator_lower_or, h=actuator_motor_h);
    # translate([actuator_min_c_c + pos - actuator_stroke - 3*actuator_nub_w/2, 0, 0])
        rotate([0, 90, 0])
        cylinder(r=actuator_upper_or, h=actuator_stroke + actuator_nub_w);
    translate([actuator_min_c_c + pos, 0, 0])
        difference() {
            cube([actuator_nub_w, actuator_nub_w, actuator_nub_w], center=true);
            cylinder(r=actuator_hole_ir, h=actuator_nub_w, center=true);
        }
    
}

actuator_bracket_th = 2.2;
actuator_bracket_l = 58;
actuator_bracket_h = 45;
actuator_bracket_w = 20;

actuator_bracket_hole_ir = 8.5/2;
actuator_bracket_hole_c_c = 36;
actuator_bracket_hole_offs = 8 + actuator_hole_ir;

module actuator_bracket_holes() {
    for (x=[0, actuator_bracket_hole_c_c])
        translate([x, 0, 0])
        cylinder(r=actuator_bracket_hole_ir, h=30, center=true, $fn=48);
}

module actuator_bracket() {
    difference() {
        translate([actuator_bracket_l/2 - actuator_bracket_hole_offs, 0, bracket_h/2])
            cube([actuator_bracket_l, actuator_bracket_w, 
            actuator_bracket_th], center=true);
        actuator_bracket_holes();
    }
}

// actuator_bracket();