#!/usr/bin/env python3
import rospy
import sys
import select
import tty
import termios
from airsim_ros.msg import RotorPWM

# ========== 配置参数 ==========
TOPIC_PWM = '/airsim_node/drone_1/rotor_pwm_cmd'
PUBLISH_RATE = 50            # 控制/显示刷新频率 (Hz)
DISPLAY_RATE = 10            # 屏幕显示刷新频率 (次/秒)

# 混控参数 (X型四旋翼，电机索引: 0-右前,1-左后,2-左前,3-右后)
THROTTLE_HOVER = 0.179         # 悬停油门初始值（根据实测调整）
PITCH_SCALE = 0.2             # 俯仰最大增量
ROLL_SCALE = 0.2              # 滚转最大增量
YAW_SCALE = 0.15              # 偏航最大增量
MAX_PWM = 1.0
MIN_PWM = 0.0

# 步长（油门精细调节）
THROTTLE_STEP = 0.0001          # 每次按键油门变化 0.01
PITCH_STEP = 0.0001             # 俯仰步长
ROLL_STEP = 0.0001
YAW_STEP = 0.0001

# ========== 键盘输入处理 ==========
class KeyReader:
    def __init__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

    def get_key(self, timeout=0):
        """非阻塞读取一个按键（字母键直接返回字符，忽略箭头键）"""
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return None
        key = sys.stdin.read(1)
        # 如果是转义序列开头（箭头键等），我们直接忽略，因为改用字母键控制油门
        if key == '\x1b':
            # 读取后续字符以清空转义序列
            seq = key
            for _ in range(2):
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    seq += sys.stdin.read(1)
                else:
                    break
            # 忽略箭头键，不返回
            return None
        return key

    def flush(self):
        """清空输入缓冲区，丢弃所有未读字符（防止长按重复）"""
        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if not rlist:
                break
            sys.stdin.read(1)

    def restore(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

# ========== 混控器 ==========
def mixer(throttle, pitch, roll, yaw):
    """
    输入: throttle [0,1], pitch/roll/yaw [-scale, scale]
    返回: [pwm0, pwm1, pwm2, pwm3]
    """
    pwm0 = throttle + pitch + roll - yaw   # 右前
    pwm1 = throttle - pitch - roll - yaw   # 左后
    pwm2 = throttle + pitch - roll + yaw   # 左前
    pwm3 = throttle - pitch + roll + yaw   # 右后
    pwms = [max(MIN_PWM, min(MAX_PWM, p)) for p in [pwm0, pwm1, pwm2, pwm3]]
    return pwms

# ========== 主函数 ==========
def main():
    rospy.init_node('keyboard_control_with_display', anonymous=True)
    pub = rospy.Publisher(TOPIC_PWM, RotorPWM, queue_size=1)
    rate = rospy.Rate(PUBLISH_RATE)

    # 尝试解锁（服务存在则调用）
    try:
        rospy.wait_for_service('/airsim_node/drone_1/arm', timeout=2.0)
        arm_srv = rospy.ServiceProxy('/airsim_node/drone_1/arm', rospy.AnyMsg)
        rospy.loginfo("Calling arm service...")
        rospy.loginfo("Arm command sent (if implemented).")
    except rospy.ROSException:
        rospy.logwarn("Arm service not available, skipping.")

    # 初始化控制量
    throttle = THROTTLE_HOVER
    pitch = 0.0
    roll = 0.0
    yaw = 0.0

    key_reader = KeyReader()
    rospy.loginfo("=== Keyboard Control with Real-time PWM Display ===")
    print("I/K: throttle ± | W/S: pitch | A/D: roll | Q/E: yaw | Space: emergency stop | R: reset throttle | P: quit")
    print("PWM values will be updated below:\n")

    display_counter = 0
    display_interval = max(1, PUBLISH_RATE // DISPLAY_RATE)

    try:
        while not rospy.is_shutdown():
            key = key_reader.get_key(timeout=0.01)
            if key:
                # ----- 油门控制：I/K -----
                if key == 'i' or key == 'I':
                    throttle = min(throttle + THROTTLE_STEP, MAX_PWM)
                elif key == 'k' or key == 'K':
                    throttle = max(throttle - THROTTLE_STEP, MIN_PWM)
                # ----- 俯仰控制：W/S -----
                elif key == 'w' or key == 'W':
                    pitch = min(pitch + PITCH_STEP, PITCH_SCALE)
                elif key == 's' or key == 'S':
                    pitch = max(pitch - PITCH_STEP, -PITCH_SCALE)
                # ----- 滚转控制：A/D -----
                elif key == 'a' or key == 'A':
                    roll = max(roll - ROLL_STEP, -ROLL_SCALE)
                elif key == 'd' or key == 'D':
                    roll = min(roll + ROLL_STEP, ROLL_SCALE)
                # ----- 偏航控制：Q/E -----
                elif key == 'q' or key == 'Q':
                    yaw = max(yaw - YAW_STEP, -YAW_SCALE)   # 逆时针
                elif key == 'e' or key == 'E':
                    yaw = min(yaw + YAW_STEP, YAW_SCALE)    # 顺时针
                # ----- 重置油门 -----
                elif key == 'r' or key == 'R':
                    throttle = THROTTLE_HOVER
                    print("\n[Throttle reset to hover]")
                # ----- 紧急停转 -----
                elif key == ' ':
                    throttle = 0.0
                    pitch = 0.0
                    roll = 0.0
                    yaw = 0.0
                    print("\n[Emergency stop]")
                # ----- 退出脚本 -----
                elif key == 'p' or key == 'P':
                    rospy.loginfo("Quit key pressed, exiting...")
                    break
                # 忽略其他键

                # 清空输入缓冲区，防止长按产生重复触发
                key_reader.flush()

            # 计算 PWM
            pwms = mixer(throttle, pitch, roll, yaw)

            # 发布消息
            msg = RotorPWM()
            msg.header.stamp = rospy.Time.now()
            msg.rotorPWM0 = pwms[0]
            msg.rotorPWM1 = pwms[1]
            msg.rotorPWM2 = pwms[2]
            msg.rotorPWM3 = pwms[3]
            pub.publish(msg)

            # 显示（降低刷新频率）
            display_counter += 1
            if display_counter >= display_interval:
                display_counter = 0
                sys.stdout.write("\033[K")
                sys.stdout.write(f"\rPWM: {pwms[0]:.3f}  {pwms[1]:.3f}  {pwms[2]:.3f}  {pwms[3]:.3f}  | "
                                 f"Thr:{throttle:.2f} Pit:{pitch:.2f} Rol:{roll:.2f} Yaw:{yaw:.2f}")
                sys.stdout.flush()

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 退出前停转电机
        stop_msg = RotorPWM()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.rotorPWM0 = stop_msg.rotorPWM1 = stop_msg.rotorPWM2 = stop_msg.rotorPWM3 = 0.0
        pub.publish(stop_msg)
        print("\nMotors stopped.")
        key_reader.restore()

if __name__ == '__main__':
    main()