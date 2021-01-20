// https://wokwi.com/playground/servo
/*
# include "kinematics.c"

void setup() {
  	Serial.begin(9600);
}

void loop() {
  	Serial.println("test");
  	test_angles_to_xyz(0, 0, 0, 0);
  	test_angles_to_xyz(180, 0, 0, 0);
  	test_angles_to_xyz(0, 0, 45, 45);
  	test_angles_to_xyz(0, 0, 90, 90);
  	test_angles_to_xyz(0, 0, -90, -90);

  	// rotation: 0.00;  x: 338.00; y: 0.00; z: 0.00
	// rotation: 180.00 x: -338.00; y: 0.00; z: 0.00
	// rotation: 0.00;  x: 208.71; y: 0.00; z: 170.71
	// rotation: 0.00;  x: 38.00; y: 0.00; z: 100.00
	// rotation: 0.00;  x: 38.00; y: 0.00; z: -100.00
	
	Serial.println();

	xyz_to_angles(0, 338, 0, 0);
	xyz_to_angles(180, -338, 0, 0);
	xyz_to_angles(0, 208.71, 0, 170.71);
	xyz_to_angles(0, 38, 0, 100);
	xyz_to_angles(0, 38, 0, -100);
	delay(10000);
  
}

void test_angles_to_xyz(float rotation, float hip_angle, float knee_angle, float ankle_angle) {
	t_leg_pos leg0;
	leg0.rotation = rotation;
	leg0.hip_angle = hip_angle;
	leg0.knee_angle = knee_angle;
	leg0.ankle_angle = ankle_angle;
	Serial.print("rotation: ");
	Serial.print(leg0.rotation);
	Serial.print("; hip_angle: ");
	Serial.print(leg0.hip_angle);
	Serial.print("; knee_angle: ");
	Serial.print(leg0.knee_angle);
	Serial.print("; ankle_angle: ");
	Serial.print(leg0.ankle_angle);
	angles_to_xyz(&leg0);
	Serial.print(" => x: ");
	Serial.print(leg0.x);
	Serial.print("; y: ");
	Serial.print(leg0.y);
	Serial.print("; z: ");
	Serial.print(leg0.z);
	Serial.println();
}

void xyz_to_angles(float rotation, float x, float y, float z) {
	t_leg_pos leg0;
	leg0.rotation = rotation;
	leg0.x = x;
	leg0.y = y;
	leg0.z = z;
	Serial.print("rotation: ");
	Serial.print(leg0.rotation);
	Serial.print(" => x: ");
	Serial.print(leg0.x);
	Serial.print("; y: ");
	Serial.print(leg0.y);
	Serial.print("; z: ");
	Serial.print(leg0.z);
	xyz_to_angles(&leg0);
	Serial.print("; hip_angle: ");
	Serial.print(leg0.hip_angle);
	Serial.print("; knee_angle: ");
	Serial.print(leg0.knee_angle);
	Serial.print("; ankle_angle: ");
	Serial.print(leg0.ankle_angle);
	Serial.println();
}
// */