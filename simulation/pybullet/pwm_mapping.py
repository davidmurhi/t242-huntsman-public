"""
Maps a joint's target angle (radians, within that joint's real URDF limits)
to the PWM pulse width (microseconds) a standard hobby servo expects.

This is the missing link between the trained policy's output and what the
ESP32 actually needs to send to drive a real servo:

    policy action (-1..1)
        -> SpiderEnv._scale_action() -> target angle in radians (within URDF limits)
        -> map_angle_to_pulse_us() [this file] -> pulse width in microseconds
        -> ESP32 PWM output -> real servo moves

PWM_MIN_US / PWM_MAX_US are a standard hobby servo range (placeholder until
Craig confirms the actual servo model - swap these for the real datasheet
values once we have them).
"""

import os
import pybullet as p

PWM_MIN_US = 500    # pulse width at the joint's lower angle limit
PWM_MAX_US = 2500   # pulse width at the joint's upper angle limit


def map_angle_to_pulse_us(angle_rad, lower_rad, upper_rad):
    """Linearly map a joint angle within [lower_rad, upper_rad] to a PWM
    pulse width within [PWM_MIN_US, PWM_MAX_US]."""
    angle_rad = max(lower_rad, min(upper_rad, angle_rad))  # clamp, just in case
    fraction = (angle_rad - lower_rad) / (upper_rad - lower_rad)
    return PWM_MIN_US + fraction * (PWM_MAX_US - PWM_MIN_US)


def get_joint_limits_from_urdf(urdf_path):
    """Pull each joint's real angle limits straight out of the URDF - same
    way SpiderEnv does - so this mapping uses the robot's actual ranges,
    not guessed numbers."""
    client = p.connect(p.DIRECT)
    robot_id = p.loadURDF(urdf_path, useFixedBase=True)

    limits = {}
    for j in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, j)
        name = info[1].decode("utf-8")
        lower, upper = info[8], info[9]
        limits[name] = (lower, upper)

    p.disconnect(client)
    return limits


if __name__ == "__main__":
    urdf_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "spider8_v5_longbody_3joint.urdf",
    )
    limits = get_joint_limits_from_urdf(urdf_path)

    examples = [
        ("l1_hip", 0.0),
        ("l1_hip", 0.5),
        ("l1_knee", -0.3),
        ("l1_ankle", 0.2),
    ]

    print(f"{'Joint':<12}{'Angle (rad)':<14}{'Range (rad)':<22}{'Pulse width (us)'}")
    for name, angle in examples:
        lower, upper = limits[name]
        pulse = map_angle_to_pulse_us(angle, lower, upper)
        print(f"{name:<12}{angle:<14.3f}[{lower:.2f}, {upper:.2f}]{'':<8}{pulse:.0f}")