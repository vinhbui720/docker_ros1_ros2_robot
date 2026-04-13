#!/usr/bin/env python3
"""
motomini_keyboard_jog.py
Keyboard jog controller for MotoMini via /joint_command (POINT_STREAMING).
Enhanced with a strict State Machine for flawless starts and stops.

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

# --- Configuration ---
PUBLISH_RATE   = 50       # Hz (50Hz is optimal for MotoROS point streaming)
SPEED_INITIAL  = 0.20     # rad/s
SPEED_MIN      = 0.01
SPEED_MAX      = 1.0
SPEED_STEP     = 0.02
MAX_LEAD_RAD   = 0.15     # Max allowed distance between command and physical robot

FORWARD_KEYS = list("123456")
REVERSE_KEYS = list("qwerty")

# --- State Machine States (Inspired by the C++ code) ---
STATE_IDLE     = 0
STATE_JOGGING  = 1
STATE_STOPPING = 2


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
        
        rospy.loginfo("Waiting for /motomini_joint_states ...")
        seed = rospy.wait_for_message(
            "/motomini_joint_states", JointState, timeout=10.0
        )
        
        self.joint_names = list(seed.name)
        self.robot_positions = list(seed.position)
        self.command_positions = list(seed.position)
        self.n_joints = len(self.joint_names)
        
        self.speed = SPEED_INITIAL
        self.active_velocities = [0.0] * self.n_joints
        self.command_latched = False
        
        # State Machine Variables
        self.state = STATE_IDLE
        self.stream_time_sec = 0.0

        self.joint_sub = rospy.Subscriber(
            "/motomini_joint_states", JointState, self._joint_state_cb, queue_size=1
        )
        rospy.loginfo("Seeded from: %s", [f"{p:.4f}" for p in self.robot_positions])

        self._trigger_traj_mode()

    # ------------------------------------------------------------------ #
    def _trigger_traj_mode(self):
        """Send current pose to /joint_path_command to arm START_TRAJ_MODE."""
        rospy.loginfo("Sending init pose to trigger trajectory mode ...")
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names  = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions       = list(self.robot_positions)
        pt.velocities      = [0.0] * self.n_joints
        pt.time_from_start = rospy.Duration(0.5)
        msg.points = [pt]
        
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

    # ------------------------------------------------------------------ #
    def _publish_point(self, positions, velocities, advance_sec=0.0):
        """Publishes a single JointTrajectoryPoint for streaming."""
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names  = self.joint_names
        
        pt = JointTrajectoryPoint()
        pt.positions        = list(positions)
        pt.velocities       = list(velocities)
        
        self.stream_time_sec += max(0.0, advance_sec)
        pt.time_from_start  = rospy.Duration.from_sec(self.stream_time_sec)
        msg.points = [pt]
        
        self.pub.publish(msg)

    # ------------------------------------------------------------------ #
    def run(self, stdscr):
        curses.cbreak()
        curses.noecho()
        stdscr.nodelay(True)
        stdscr.keypad(True)

        rate = rospy.Rate(PUBLISH_RATE)
        
        # Initialize the last loop time for variable dt calculation
        last_loop_time = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            
            # Calculate the actual elapsed time since the last loop
            dt_actual = (now - last_loop_time).to_sec()
            last_loop_time = now
            
            # Safety clamp: Limit dt to 0.1 seconds to prevent massive 
            # position jumps if the ROS node momentarily hangs.
            dt = min(dt_actual, 0.1)
            
            key_motion_updated = False

            # 1. Process Input
            try:
                while True:
                    ch = stdscr.getch()
                    if ch == -1: break
                    if ch == 27: return # ESC to quit
                    if ch < 32 or ch > 126: continue
                    
                    key = chr(ch)
                    if key in FORWARD_KEYS:
                        self.active_velocities = [0.0] * self.n_joints
                        self.active_velocities[FORWARD_KEYS.index(key)] = self.speed
                        self.command_latched = True
                        key_motion_updated = True
                    elif key in REVERSE_KEYS:
                        self.active_velocities = [0.0] * self.n_joints
                        self.active_velocities[REVERSE_KEYS.index(key)] = -self.speed
                        self.command_latched = True
                        key_motion_updated = True
                    elif key == "=":
                        self.speed = min(SPEED_MAX, round(self.speed + SPEED_STEP, 4))
                    elif key == "-":
                        self.speed = max(SPEED_MIN, round(self.speed - SPEED_STEP, 4))
                    elif key == " ":
                        self.active_velocities = [0.0] * self.n_joints
                        self.command_latched = False
            except Exception:
                pass

            # Update speeds if general speed was changed while moving
            if self.command_latched and not key_motion_updated:
                for i, v in enumerate(self.active_velocities):
                    if v > 0.0: self.active_velocities[i] = self.speed
                    elif v < 0.0: self.active_velocities[i] = -self.speed

            target_velocities = list(self.active_velocities) if self.command_latched else [0.0] * self.n_joints
            is_input_moving = any(abs(v) > 1e-9 for v in target_velocities)

            with self.state_lock:
                actual_positions = list(self.robot_positions)

            # 2. State Machine Logic
            if self.state == STATE_IDLE:
                if is_input_moving:
                    # Transition to JOGGING: Seed command to exactly where the robot currently is
                    self.stream_time_sec = 0.0
                    self.command_positions = list(actual_positions)
                    self.state = STATE_JOGGING

            elif self.state == STATE_JOGGING:
                if is_input_moving:
                    # Integrate velocities into positions
                    for i in range(self.n_joints):
                        self.command_positions[i] += target_velocities[i] * dt
                        
                        # Math Clamping Formula: Prevent runaway buffers
                        lead = self.command_positions[i] - actual_positions[i]
                        if lead > MAX_LEAD_RAD:
                            self.command_positions[i] = actual_positions[i] + MAX_LEAD_RAD
                        elif lead < -MAX_LEAD_RAD:
                            self.command_positions[i] = actual_positions[i] - MAX_LEAD_RAD
                            
                    self._publish_point(self.command_positions, target_velocities, dt)
                else:
                    # Input stopped, transition to STOPPING
                    self.state = STATE_STOPPING

            if self.state == STATE_STOPPING:
                # To stop cleanly without a rebound, publish V=0 at the LAST COMMANDED position.
                self._publish_point(self.command_positions, [0.0] * self.n_joints, dt)
                self.state = STATE_IDLE  # Rest until next input

            # 3. UI Update
            try:
                stdscr.erase()
                stdscr.addstr(0, 0, "=== MotoMini State Machine Jog ===  (ESC to quit)")
                stdscr.addstr(1, 0, f"  Speed : {self.speed:.3f} rad/s   ( - = decr,  = = incr,  SPACE = stop )")
                state_str = ["IDLE", "JOGGING", "STOPPING"][self.state]
                stdscr.addstr(2, 0, f"  State : {state_str}")
                stdscr.addstr(3, 0, "  Joint  | Fwd | Rev |   Position (rad)   | Joint name")
                stdscr.addstr(4, 0, "  -------+-----+-----+--------------------+-----------")
                for i in range(self.n_joints):
                    moving_str = ""
                    if target_velocities[i] > 0:   moving_str = ">>>"
                    elif target_velocities[i] < 0: moving_str = "<<<"
                    
                    actual_pos = actual_positions[i]
                    stdscr.addstr(
                        5 + i, 0,
                        f"  J{i+1}     |  {FORWARD_KEYS[i]}  |  {REVERSE_KEYS[i]}  |  {actual_pos:+.5f} rad  {moving_str:<4}  {self.joint_names[i]}"
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