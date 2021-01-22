import math

# Scratchpad for simulating walking robot

BODY_PIVOT_R = 100
HIP_L = 38
HIP_DZ = 0
LEG_L = 100
FOOT_L = 100
ANKLE_BIAS = 45
PI = math.pi  # 3.141592
DEG2RAD = PI / 180.0;
RAD2DEG = 180.0 / PI;

# // Data types
# typedef struct {
#     float rotation;
#     float hip_angle;
#     float knee_angle;
#     float ankle_angle;
#     float x;
#     float y;
#     float z;
# } t_leg_pos;

class t_leg_pos:
    def __init__(self, rotation=0.0, hip_angle=0.0, knee_angle=0.0, ankle_angle=0.0, x=0.0, y=0.0, z=0.0):
        self.rotation = float(rotation)
        self.hip_angle = float(hip_angle)
        self.knee_angle = float(knee_angle)
        self.ankle_angle = float(ankle_angle)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __repr__(self):
        return (
            f"t_leg_pos(rotation={self.rotation}, hip_angle={self.hip_angle}, knee_angle={self.knee_angle}, "
            f"ankle_angle={self.ankle_angle}, x={self.x}, y={self.y}, z={self.z})")

    def __eq__(self, other):
        return (
            abs(self.rotation - other.rotation) < 0.1 and 
            # abs(self.hip_angle - other.hip_angle) < 0.1 and
            # abs(self.knee_angle - other.knee_angle) < 0.1 and
            # abs(self.ankle_angle - other.ankle_angle) < 0.1 and
            abs(self.x - other.x)  < 0.1 and
            abs(self.y - other.y)  < 0.1 and
            abs(self.z - other.z) < 0.1)

def angles_to_xyz(leg):
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

    leg.x += extension_x * math.cos(DEG2RAD * (leg.rotation - leg.hip_angle))
    leg.y += extension_x * math.sin(DEG2RAD * (leg.rotation - leg.hip_angle))
    leg.z -= extension_z
    print(f"{locals()}")
    return leg

def xyz_to_angles(leg):
    """Prefer vertical feet (ankle_angle + ANKLE_BIAS ~= -knee_angle)
    """

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

def test_outstretched():
    # print("Outstretched")
    _test_both_ways(0, 0, 0, -ANKLE_BIAS, 338, 0, 0)
    _test_angle_xyz_angle()

def test_rear_outstretched():
    # print("Rear leg, Outstretched")
    _test_both_ways(180, 0, 0, -ANKLE_BIAS, -338, 0, 0)
    _test_angle_xyz_angle(rotation=180)
  

def test_tall_stand():
    _test_angle_xyz_angle(0, 0, 90, -45)  

def test_low_stand():
    _test_angle_xyz_angle(0, 0, -45, 90)

def test_med_stand():
    _test_angle_xyz_angle(0, 0, 0, 45)

# def test_curl_over():
#     _test_both_ways(0, 0, -90, -90, 38 + 2 * FOOT_L * math.sqrt(2), 0, LEG_L + 2 * FOOT_L * math.sqrt(2))


# def test_curl_under():
#     _test_both_ways(0, 0, 90, 45, 38, 0, -100)


def test_curl_up():
    _test_angle_xyz_angle(0, 0, 0, -90)

def test_curl_down():
    _test_angle_xyz_angle(0, 0, 0, 90)

def test_xyz_angle_xyz(rotation=0, x=200, y=30, z=-100):
    xyz_to_a = _test_xyz_to_angles(rotation, x, y, z)
    a_to_xyz = _test_angles_to_xyz(rotation, xyz_to_a.hip_angle, xyz_to_a.knee_angle, xyz_to_a.ankle_angle)
    assert a_to_xyz == xyz_to_a



def _test_angle_xyz_angle(rotation=0, hip_angle=0, knee_angle=0, ankle_angle=-45):
    a_to_xyz = _test_angles_to_xyz(rotation, hip_angle, knee_angle, ankle_angle)
    xyz_to_a = _test_xyz_to_angles(rotation, a_to_xyz.x, a_to_xyz.y, a_to_xyz.z)
    assert a_to_xyz == xyz_to_a
    return a_to_xyz
    # // rotation: 0.00;  x: 338.00; y: 0.00; z: 0.00
    # // rotation: 180.00 x: -338.00; y: 0.00; z: 0.00
    # // rotation: 0.00;  x: 208.71; y: 0.00; z: 170.71
    # // rotation: 0.00;  x: 38.00; y: 0.00; z: 100.00
    # // rotation: 0.00;  x: 38.00; y: 0.00; z: -100.00
    
    # test_xyz_to_angles(0, 338, 0, 0)
    # test_xyz_to_angles(180, -338, 0, 0)
    # test_xyz_to_angles(0, 208.71, 0, 170.71)
    # test_xyz_to_angles(0, 38, 0, 100)
    # test_xyz_to_angles(0, 38, 0, -100)

def _test_both_ways(rotation, hip_angle, knee_angle, ankle_angle, x, y, z):
    a_to_xyz = _test_angles_to_xyz(rotation, hip_angle, knee_angle, ankle_angle)
    xyz_to_a = _test_xyz_to_angles(rotation, x, y, z)

    assert a_to_xyz == xyz_to_a

def _test_angles_to_xyz(rotation, hip_angle, knee_angle, ankle_angle):
    leg0 = t_leg_pos()
    leg0.rotation = rotation
    leg0.hip_angle = hip_angle
    leg0.knee_angle = knee_angle
    leg0.ankle_angle = ankle_angle
    print(f"Input leg:  {leg0}")
    leg0 = angles_to_xyz(leg0)
    print(f"Output leg: {leg0}")
    return leg0


def _test_xyz_to_angles(rotation, x, y, z):
    leg0 = t_leg_pos()
    leg0.rotation = rotation;
    leg0.x = x;
    leg0.y = y;
    leg0.z = z;
    print(f"Input leg:  {leg0}")
    leg0 = xyz_to_angles(leg0);
    print(f"Output leg: {leg0}")
    return leg0



def test_experiment1():
    hip_angle = 0
    knee_angle = -90
    ankle_angle = 0 
    x = 208.711
    y = 0
    z = 170.711
    a_to_xyz = _test_angle_xyz_angle(0, hip_angle, knee_angle, ankle_angle)
    assert abs(a_to_xyz.x - x) < 0.1
    assert abs(a_to_xyz.y - y) < 0.1
    assert abs(a_to_xyz.z - z) < 0.1

def test_experiment2():
    hip_angle = 0
    knee_angle = -20
    ankle_angle = 50
    x = 257.851
    y = 0
    z = -62.3906
    a_to_xyz = _test_angle_xyz_angle(0, hip_angle, knee_angle, ankle_angle)
    assert abs(a_to_xyz.x - x) < 0.1
    assert abs(a_to_xyz.y - y) < 0.1
    assert abs(a_to_xyz.z - z) < 0.1

def test_experiment3():
    hip_angle = 10
    knee_angle = -20
    ankle_angle = 50
    x = 255.453 
    y = -27.4106 
    z = -62.3906
    a_to_xyz = _test_angle_xyz_angle(0, hip_angle, knee_angle, ankle_angle)
    assert abs(a_to_xyz.x - x) < 0.1
    assert abs(a_to_xyz.y - y) < 0.1
    assert abs(a_to_xyz.z - z) < 0.1

def test_experiment4():
    hip_angle = 10
    knee_angle = -30
    ankle_angle = -90
    x = 248.198 
    y = -26.1313
    z = 146.593
    a_to_xyz = _test_angle_xyz_angle(0, hip_angle, knee_angle, ankle_angle)
    assert abs(a_to_xyz.x - x) < 0.1
    assert abs(a_to_xyz.y - y) < 0.1
    assert abs(a_to_xyz.z - z) < 0.1

if __name__ == '__main__':
    test_all()
