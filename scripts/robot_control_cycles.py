#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

# cmd_vel publish rate (Hz)
CMD_RATE_HZ = 20

# acceleration ramp time at segment start/end (seconds)
RAMP_TIME = 0.3

# how many zero-twist messages to publish on stop (defeats dropped packets)
STOP_BURST = 5


def publish_cmd(pub, vx=0.0, vy=0.0, wz=0.0):
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = wz
    pub.publish(msg)


def publish_stop(pub, rate):
    """Publish zero twist several times to ensure the robot actually stops."""
    for _ in range(STOP_BURST):
        publish_cmd(pub, 0.0, 0.0, 0.0)
        rate.sleep()


def wait_for_subscriber(pub, timeout=5.0):
    """Block until at least one subscriber connects, or timeout elapses."""
    start = rospy.Time.now().to_sec()
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - start > timeout:
            rospy.logwarn("No subscriber on /cmd_vel after %.1fs; continuing anyway.", timeout)
            return False
        rospy.sleep(0.05)
    return True


def ramped_velocity(elapsed, duration, target_speed):
    """
    Trapezoidal velocity profile: ramp up over RAMP_TIME, cruise, ramp down over RAMP_TIME.
    If the segment is too short to fit two ramps, shrink them symmetrically.
    """
    ramp = min(RAMP_TIME, duration / 2.0)
    if elapsed < ramp:
        return target_speed * (elapsed / ramp)
    if elapsed > duration - ramp:
        remaining = max(0.0, duration - elapsed)
        return target_speed * (remaining / ramp)
    return target_speed


def move_for_duration(pub, rate, duration, vx=0.0, vy=0.0, wz=0.0):
    """Drive with a trapezoidal profile on whichever axis is nonzero."""
    start = rospy.Time.now().to_sec()
    target_vx, target_vy, target_wz = vx, vy, wz

    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - start
        if elapsed >= duration:
            break

        # scale each component by the ramp factor so direction is preserved
        scale_x = ramped_velocity(elapsed, duration, abs(target_vx)) / abs(target_vx) if target_vx else 0.0
        scale_y = ramped_velocity(elapsed, duration, abs(target_vy)) / abs(target_vy) if target_vy else 0.0
        scale_z = ramped_velocity(elapsed, duration, abs(target_wz)) / abs(target_wz) if target_wz else 0.0

        publish_cmd(
            pub,
            vx=target_vx * scale_x,
            vy=target_vy * scale_y,
            wz=target_wz * scale_z,
        )
        rate.sleep()

    publish_stop(pub, rate)


def move_linear(pub, rate, signed_distance, speed, axis="x"):
    if speed <= 0:
        raise ValueError("speed must be > 0")

    direction = 1.0 if signed_distance >= 0 else -1.0
    duration = abs(signed_distance) / speed

    if axis == "x":
        move_for_duration(pub, rate, duration, vx=direction * speed)
    elif axis == "y":
        move_for_duration(pub, rate, duration, vy=direction * speed)
    else:
        raise ValueError("axis must be 'x' or 'y'")


DIRECTION_AXIS = {
    "forward":  ("x",  1.0),
    "backward": ("x", -1.0),
    "left":     ("y",  1.0),
    "right":    ("y", -1.0),
}

OPPOSITE = {
    "forward":  "backward",
    "backward": "forward",
    "left":     "right",
    "right":    "left",
}


def execute_single(pub, rate, direction, distance, speed):
    direction = direction.strip().lower()
    if direction not in DIRECTION_AXIS:
        raise ValueError("direction must be one of: forward, backward, left, right")

    axis, sign = DIRECTION_AXIS[direction]
    move_linear(pub, rate, sign * distance, speed, axis=axis)


def execute_sequence(pub, rate, direction, distance, speed, cycles):
    if direction not in OPPOSITE:
        raise ValueError("invalid direction")

    first = direction
    second = OPPOSITE[direction]

    for i in range(cycles):
        if rospy.is_shutdown():
            break
        rospy.loginfo("Cycle %d/%d: %s %.2f m", i + 1, cycles, first, distance)
        execute_single(pub, rate, first, distance, speed)

        if rospy.is_shutdown():
            break
        rospy.loginfo("Cycle %d/%d: %s %.2f m", i + 1, cycles, second, distance)
        execute_single(pub, rate, second, distance, speed)


def get_user_motion():
    try:
        direction = input("\nEnter direction (forward/backward/left/right) or q to quit: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        return None

    if direction == "q":
        return None

    if direction not in DIRECTION_AXIS:
        print("Invalid direction.")
        return "invalid"

    try:
        distance = float(input("Enter distance in meters: ").strip())
        speed = float(input("Enter speed in m/s: ").strip())
        cycles = int(input("Enter number of cycles: ").strip())
    except (ValueError, EOFError):
        print("Invalid numeric input.")
        return "invalid"
    except KeyboardInterrupt:
        return None

    if distance <= 0:
        print("Distance must be > 0")
        return "invalid"
    if speed <= 0:
        print("Speed must be > 0")
        return "invalid"
    if cycles <= 0:
        print("Cycles must be > 0")
        return "invalid"

    return direction, distance, speed, cycles


def main():
    rospy.init_node("interactive_paired_motion_sequence")

    # queue_size=1 for streaming setpoints so stale commands don't pile up
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(CMD_RATE_HZ)

    wait_for_subscriber(pub, timeout=5.0)

    print("Paired motion controller started.")
    print("forward  -> forward then backward")
    print("backward -> backward then forward")
    print("left     -> left then right")
    print("right    -> right then left")

    try:
        while not rospy.is_shutdown():
            motion = get_user_motion()

            if motion is None:
                print("Quitting.")
                break

            if motion == "invalid":
                continue

            direction, distance, speed, cycles = motion
            execute_sequence(pub, rate, direction, distance, speed, cycles)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            publish_stop(pub, rate)
        except Exception:
            pass


if __name__ == "__main__":
    main()