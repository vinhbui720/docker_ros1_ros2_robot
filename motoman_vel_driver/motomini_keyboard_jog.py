#!/usr/bin/env python3
import rospy
import curses
import threading
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ================================
# CONFIG
# ================================
PUBLISH_RATE = 50
DT = 1.0 / PUBLISH_RATE

SPEED = 0.3

JOINT_LIMITS = [
    (-2.9670,  2.9670, 5.4977),
    (-1.4835,  1.5707, 5.4977),
    (-0.8726,  1.5707, 7.3304),
    (-2.4434,  2.4434, 10.4719),
    (-0.5235,  3.6651, 10.4719),
    (-6.2831,  6.2831, 10.4719),
]

FORWARD_KEYS = list("123456")
REVERSE_KEYS = list("qwerty")

STATE_IDLE = 0
STATE_POSE_FOLLOW = 1
STATE_STOP = 2


class Jogger:
    def __init__(self):
        rospy.init_node("stateless_jogger")

        self.pub = rospy.Publisher("/joint_command", JointTrajectory, queue_size=5)
        self.init_pub = rospy.Publisher("/joint_path_command", JointTrajectory, queue_size=1)

        self.lock = threading.Lock()

        msg = rospy.wait_for_message("/motomini_joint_states", JointState)
        self.joint_names = list(msg.name)
        self.n = len(self.joint_names)

        self.actual_pos = list(msg.position[:self.n])
        self.actual_vel = [0.0] * self.n

        self.desired_vel = [0.0] * self.n

        self.state = STATE_IDLE
        self.t_start = rospy.Time.now()
        self.t_last_input = rospy.Time.now()

        rospy.Subscriber("/motomini_joint_states", JointState, self.cb_joint)

        self.arm()

    # ================================
    # ARM TRAJECTORY MODE
    # ================================
    def arm(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = list(self.actual_pos)
        pt.velocities = [0.0] * self.n
        pt.time_from_start = rospy.Duration(0.5)

        msg.points = [pt]

        rospy.sleep(0.5)
        self.init_pub.publish(msg)
        rospy.sleep(1.0)

    # ================================
    # SEED FUNCTION (NOT ENABLED)
    # ================================
    def seed(self):
        """
        Send initial point EXACTLY at current position.
        Use when switching to controllers that require strict start pose.
        """
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = list(self.actual_pos)
        pt.velocities = [0.0] * self.n
        pt.time_from_start = rospy.Duration(0.0)

        msg.points = [pt]
        self.pub.publish(msg)

    # ================================
    def cb_joint(self, msg):
        with self.lock:
            self.actual_pos = list(msg.position[:self.n])

    # ================================
    def send(self, pos, vel, t):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = pos
        pt.velocities = vel
        pt.time_from_start = rospy.Duration.from_sec(t)

        msg.points = [pt]
        self.pub.publish(msg)

    # ================================
    def run(self, stdscr):
        curses.cbreak()
        curses.noecho()
        stdscr.nodelay(True)

        rate = rospy.Rate(PUBLISH_RATE)

        while not rospy.is_shutdown():
            # ========================
            # INPUT
            # ========================
            try:
                while True:
                    ch = stdscr.getch()
                    if ch == -1:
                        break

                    c = chr(ch)

                    if c in FORWARD_KEYS:
                        idx = FORWARD_KEYS.index(c)
                        self.desired_vel = [0.0]*self.n
                        self.desired_vel[idx] = SPEED
                        self.t_last_input = rospy.Time.now()

                    elif c in REVERSE_KEYS:
                        idx = REVERSE_KEYS.index(c)
                        self.desired_vel = [0.0]*self.n
                        self.desired_vel[idx] = -SPEED
                        self.t_last_input = rospy.Time.now()

                    elif c == " ":
                        self.desired_vel = [0.0]*self.n
                        self.state = STATE_STOP

            except:
                pass

            with self.lock:
                actual = list(self.actual_pos)

            moving = any(abs(v) > 1e-6 for v in self.desired_vel)

            # ========================
            # STATE TRANSITION
            # ========================
            if self.state == STATE_IDLE and moving:
                self.state = STATE_POSE_FOLLOW
                self.t_start = rospy.Time.now()
                # self.seed()  # <<< NOT ENABLED (for MotoPlus test)

            elif self.state == STATE_POSE_FOLLOW:
                dt_cb = (rospy.Time.now() - self.t_last_input).to_sec()
                if dt_cb > 0.5:
                    self.state = STATE_IDLE

            elif self.state == STATE_STOP:
                self.state = STATE_IDLE

            # ========================
            # EXECUTION
            # ========================
            if self.state == STATE_POSE_FOLLOW:
                now = rospy.Time.now()
                t = (now - self.t_start).to_sec()

                new_pos = list(actual)
                new_vel = [0.0]*self.n

                for i in range(self.n):
                    v = self.desired_vel[i]

                    # clamp vel
                    vmax = JOINT_LIMITS[i][2]
                    v = max(-vmax, min(vmax, v))

                    new_pos[i] += v * DT

                    # clamp pos
                    lo, hi = JOINT_LIMITS[i][0], JOINT_LIMITS[i][1]
                    new_pos[i] = max(lo, min(hi, new_pos[i]))

                    new_vel[i] = v

                self.send(new_pos, new_vel, t)

            # ========================
            # UI
            # ========================
            try:
                state_str = ["IDLE", "FOLLOW", "STOP"][self.state]
                stdscr.erase()
                stdscr.addstr(0, 0, f"State: {state_str}")
                for i in range(self.n):
                    stdscr.addstr(2+i, 0,
                        f"J{i+1}: pos={actual[i]:+.3f} vel={self.desired_vel[i]:+.2f}")
                stdscr.refresh()
            except:
                pass

            rate.sleep()


def main():
    jogger = Jogger()
    curses.wrapper(jogger.run)


if __name__ == "__main__":
    main()