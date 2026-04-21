#!/usr/bin/env python3
import atexit
import select
import sys
import termios
import threading
import tty

import rospy
from airsim_ros.msg import VelCmd

DEFAULT_TOPIC_VEL = '/airsim_node/drone_1/vel_body_cmd'
PUBLISH_RATE = 50
XY_ACCEL_LIMIT = 4

MAX_LINEAR_VEL = 6.0
MAX_UPWARD_VEL = 1.5
MAX_ANGULAR_VEL = 6.0

LIN_STEP = 0.2
ANG_STEP = 0.2


class KeyReader:
    def __init__(self):
        if not sys.stdin.isatty():
            raise RuntimeError('stdin is not a TTY')
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        atexit.register(self.restore)

    def get_key(self, timeout=0.1):
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return None
        key = sys.stdin.read(1)
        if key == '\x03':
            raise KeyboardInterrupt
        if key == '\x1b':
            self._drain_escape_sequence()
            return None
        return key.lower()

    def _drain_escape_sequence(self):
        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            if not rlist:
                break
            sys.stdin.read(1)

    def restore(self):
        if getattr(self, 'old_settings', None) is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            self.old_settings = None


class VelocityState:
    def __init__(self):
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        self.running = True
        self.lock = threading.Lock()

    def apply_key(self, key):
        with self.lock:
            if key == 'w':
                self.vx = min(self.vx + LIN_STEP, MAX_LINEAR_VEL)
            elif key == 's':
                self.vx = max(self.vx - LIN_STEP, -MAX_LINEAR_VEL)
            elif key == 'a':
                self.vy = max(self.vy - LIN_STEP, -MAX_LINEAR_VEL)
            elif key == 'd':
                self.vy = min(self.vy + LIN_STEP, MAX_LINEAR_VEL)
            elif key == 'i':
                self.vz = min(self.vz + LIN_STEP, MAX_UPWARD_VEL)
            elif key == 'k':
                self.vz = max(self.vz - LIN_STEP, -MAX_UPWARD_VEL)
            elif key == 'q':
                self.yaw_rate = max(self.yaw_rate - ANG_STEP, -MAX_ANGULAR_VEL)
            elif key == 'e':
                self.yaw_rate = min(self.yaw_rate + ANG_STEP, MAX_ANGULAR_VEL)
            elif key == ' ':
                self.vx = self.vy = self.vz = self.yaw_rate = 0.0
            elif key == 'p':
                self.running = False

    def snapshot(self):
        with self.lock:
            return self.vx, self.vy, self.vz, self.yaw_rate, self.running

    def stop(self):
        with self.lock:
            self.running = False


def publisher_loop(pub, state, xy_accel_limit):
    rate = rospy.Rate(PUBLISH_RATE)
    while not rospy.is_shutdown():
        vx, vy, vz, yaw_rate, running = state.snapshot()
        if not running:
            break

        cmd = VelCmd()
        cmd.header.stamp = rospy.Time.now()
        cmd.vx = vx
        cmd.vy = vy
        cmd.vz = vz
        cmd.yawRate = yaw_rate
        cmd.va = xy_accel_limit
        cmd.stop = 0
        pub.publish(cmd)

        sys.stdout.write(
            f'\r指令速度 -> 前后:{vx:.1f} 左右:{vy:.1f} 上下:{vz:.1f} 自旋:{yaw_rate:.1f}   '
        )
        sys.stdout.flush()
        rate.sleep()


def main():
    rospy.init_node('keyboard_vel_control', anonymous=True)
    topic_vel = rospy.get_param('~topic_vel', DEFAULT_TOPIC_VEL)
    xy_accel_limit = int(rospy.get_param('~xy_accel_limit', XY_ACCEL_LIMIT))
    pub = rospy.Publisher(topic_vel, VelCmd, queue_size=1)

    state = VelocityState()
    key_reader = KeyReader()
    pub_thread = threading.Thread(
        target=publisher_loop,
        args=(pub, state, xy_accel_limit),
        daemon=True,
    )
    pub_thread.start()

    print('=== 无人机速度控制模式 (Velocity Control) ===')
    print('W/S: 前/后 | A/D: 左/右 | I/K: 上/下 | Q/E: 自旋')
    print('Space: 悬停(速度清零) | P: 退出 | Ctrl+C: 强制退出')

    try:
        while not rospy.is_shutdown():
            key = key_reader.get_key()
            if key is None:
                continue
            state.apply_key(key)
            _, _, _, _, running = state.snapshot()
            if not running:
                break
    except KeyboardInterrupt:
        pass
    finally:
        state.stop()
        pub_thread.join(timeout=1.0)

        stop_cmd = VelCmd()
        stop_cmd.header.stamp = rospy.Time.now()
        stop_cmd.stop = 1
        pub.publish(stop_cmd)

        key_reader.restore()
        print('\n已停止控制并恢复终端设置。')


if __name__ == '__main__':
    main()
