#!/usr/bin/env python3
"""
motomini_keyboard_jog.py
Keyboard jog controller for MotoMini via /joint_command (POINT_STREAMING).

Keys:
  1 2 3 4 5 6  -> joint 1-6 forward  (+)
  q w e r t y  -> joint 1-6 reverse  (-)
  -            -> decrease speed
  =            -> increase speed
  ESC / Ctrl+C -> quit
"""

import sys
import curses
import threading
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PUBLISH_RATE   = 20       # Hz
SPEED_INITIAL  = 0.20     # rad/s
SPEED_MIN      = 0.01
SPEED_MAX      = 1.0
SPEED_STEP     = 0.02
MAX_LEAD_RAD   = 0.20    # max command lead ahead of measured joint state

FORWARD_KEYS = list("123456")
REVERSE_KEYS = list("qwerty")


class KeyboardJogger:
    def __init__(self):
        rospy.init_node("motomini_keyboard_jog", anonymous=False)

        self.pub = rospy.Publisher(
            "/joint_command", JointTrajectory, queue_size=1
        )
        self.init_pub = rospy.Publisher(
            "/joint_path_command", JointTrajectory, queue_size=1
        )
        self.state_lock = threading.Lock()
        self.last_joint_state_stamp = rospy.Time(0)

        rospy.loginfo("Waiting for /motomini_joint_states ...")
        seed = rospy.wait_for_message(
            "/motomini_joint_states", JointState, timeout=10.0
        )
        self.joint_names = list(seed.name)
        self.positions   = list(seed.position)
        self.robot_positions = list(seed.position)
        self.command_positions = list(seed.position)
        self.n_joints    = len(self.joint_names)
        self.speed       = SPEED_INITIAL
        self.active_velocities = [0.0] * self.n_joints
        self.last_cmd_time = rospy.Time(0)
        self.was_streaming = False
        self.command_latched = False

        self.joint_sub = rospy.Subscriber(
            "/motomini_joint_states", JointState, self._joint_state_cb, queue_size=1
        )
        rospy.loginfo("Seeded from: %s", [f"{p:.4f}" for p in self.positions])

        self._trigger_traj_mode()

    # ------------------------------------------------------------------ #
    def _trigger_traj_mode(self):
        """Send current pose to /joint_path_command to arm START_TRAJ_MODE."""
        rospy.loginfo("Sending init pose to /joint_path_command to trigger traj mode ...")
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names  = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions       = list(self.positions)
        pt.velocities      = [0.0] * self.n_joints
        pt.time_from_start = rospy.Duration(0.5)
        msg.points = [pt]
        # Wait for the subscriber on the other end to be ready
        rospy.sleep(0.5)
        self.init_pub.publish(msg)
        rospy.loginfo("Init pose sent. Waiting for traj mode to arm ...")
        rospy.sleep(1.0)

    # ------------------------------------------------------------------ #
    def _joint_state_cb(self, msg):
        if len(msg.position) < self.n_joints:
            return
        with self.state_lock:
            self.robot_positions = list(msg.position[: self.n_joints])
            self.last_joint_state_stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()

    # ------------------------------------------------------------------ #
    def _publish(self, positions, velocities):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names  = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions        = list(positions)
        pt.velocities       = list(velocities)
        pt.time_from_start  = rospy.Duration(0)
        msg.points = [pt]
        self.pub.publish(msg)

    # ------------------------------------------------------------------ #
    def run(self, stdscr):
        curses.cbreak()
        curses.noecho()
        stdscr.nodelay(True)   # non-blocking getch
        stdscr.keypad(True)

        rate = rospy.Rate(PUBLISH_RATE)
        dt   = 1.0 / PUBLISH_RATE

        # Do not continuously publish zero commands at startup.
        # We seed only when motion starts, so POINT_STREAMING can return to IDLE when idle.

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            key_motion_updated = False

            # Drain all pending key events this cycle
            try:
                while True:
                    ch = stdscr.getch()
                    if ch == -1:
                        break
                    if ch == 27:                     # ESC
                        return
                    if ch < 32 or ch > 126:
                        continue
                    key = chr(ch)
                    if key in FORWARD_KEYS:
                        self.active_velocities = [0.0] * self.n_joints
                        self.active_velocities[FORWARD_KEYS.index(key)] = self.speed
                        self.last_cmd_time = now
                        self.command_latched = True
                        key_motion_updated = True
                    elif key in REVERSE_KEYS:
                        self.active_velocities = [0.0] * self.n_joints
                        self.active_velocities[REVERSE_KEYS.index(key)] = -self.speed
                        self.last_cmd_time = now
                        self.command_latched = True
                        key_motion_updated = True
                    elif key == "=":
                        self.speed = min(SPEED_MAX, round(self.speed + SPEED_STEP, 4))
                    elif key == "-":
                        self.speed = max(SPEED_MIN, round(self.speed - SPEED_STEP, 4))
                    elif key == " ":
                        self.active_velocities = [0.0] * self.n_joints
                        self.command_latched = False
                        self.last_cmd_time = now
            except Exception:
                pass

            # Latch command until SPACE is pressed. This is more reliable in docker terminals.
            if self.command_latched:
                velocities = list(self.active_velocities)
            else:
                velocities = [0.0] * self.n_joints

            # If speed changed this cycle, update active command magnitude.
            if not key_motion_updated and any(v != 0.0 for v in self.active_velocities):
                for i, v in enumerate(self.active_velocities):
                    if v > 0.0:
                        self.active_velocities[i] = self.speed
                    elif v < 0.0:
                        self.active_velocities[i] = -self.speed

            moving = any(abs(v) > 1e-9 for v in velocities)

            # Build commands from measured robot state to avoid running ahead.
            with self.state_lock:
                base_positions = list(self.robot_positions)

            if moving:
                if not self.was_streaming:
                    # First point of a new stream: seed exactly at current pose.
                    self.command_positions = list(base_positions)
                    self._publish(base_positions, [0.0] * self.n_joints)
                    rate.sleep()

                for i in range(self.n_joints):
                    self.command_positions[i] += velocities[i] * dt

                    # Keep command close to measured state to avoid runaway buffering.
                    lead = self.command_positions[i] - base_positions[i]
                    if lead > MAX_LEAD_RAD:
                        self.command_positions[i] = base_positions[i] + MAX_LEAD_RAD
                    elif lead < -MAX_LEAD_RAD:
                        self.command_positions[i] = base_positions[i] - MAX_LEAD_RAD
                self._publish(self.command_positions, velocities)
                self.was_streaming = True
            else:
                if self.was_streaming:
                    # Send one zero-velocity point to stop cleanly, then go quiet.
                    self._publish(base_positions, [0.0] * self.n_joints)
                self.was_streaming = False

            # ---- UI ----
            try:
                stdscr.erase()
                stdscr.addstr(0, 0, "=== MotoMini Keyboard Jog ===  (ESC / Ctrl-C to quit)")
                stdscr.addstr(1, 0, f"  Speed : {self.speed:.3f} rad/s   ( - = decrease    = = increase, SPACE = stop )")
                stdscr.addstr(3, 0, "  Joint  | Fwd | Rev |   Position (rad)   | Joint name")
                stdscr.addstr(4, 0, "  -------+-----+-----+--------------------+-----------")
                for i in range(self.n_joints):
                    moving = ""
                    if velocities[i] > 0:   moving = ">>>"
                    elif velocities[i] < 0: moving = "<<<"
                    with self.state_lock:
                        actual_pos = self.robot_positions[i]
                    stdscr.addstr(
                        5 + i, 0,
                        f"  J{i+1}     |  {FORWARD_KEYS[i]}  |  {REVERSE_KEYS[i]}  |  {actual_pos:+.5f} rad  {moving:<4}  {self.joint_names[i]}"
                    )
                stdscr.refresh()
            except curses.error:
                pass

            rate.sleep()


def main():
    try:
        jogger = KeyboardJogger()
        curses.wrapper(jogger.run)
    except rospy.ROSException as e:
        print(f"ROS error: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()