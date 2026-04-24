#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <airsim_ros/VelCmd.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <cmath>

// 独立的 PID 控制器类
class PID {
public:
    double kp, ki, kd;
    double max_out;
    double max_integral;
    double integral = 0;
    double last_err = 0;
    bool first_run = true;

    PID() : kp(0), ki(0), kd(0), max_out(0), max_integral(0) {}

    void reset() {
        integral = 0;
        last_err = 0;
        first_run = true;
    }

    double compute(double setpoint, double measure, double dt) {
        double err = setpoint - measure;
        double p_term = kp * err;

        integral += err * dt;
        if (max_integral > 0) {
            integral = std::clamp(integral, -max_integral, max_integral);
        }
        double i_term = ki * integral;

        double d_term = 0;
        if (!first_run && dt > 0) {
            d_term = kd * (err - last_err) / dt;
        }
        last_err = err;
        first_run = false;

        return std::clamp(p_term + i_term + d_term, -max_out, max_out);
    }
};

class PositionControllerNode {
public:
    static constexpr uint8_t FLAG_TRACK = 0;
    static constexpr uint8_t FLAG_BRAKE_FOR_TURN = 101;
    static constexpr uint8_t FLAG_ROTATE_FOR_TURN = 102;

    PositionControllerNode() : nh("~"), last_control_time(ros::Time::now()) {
        loadParams();

        // 订阅目标、反馈，发布速度指令
        pos_cmd_sub = nh.subscribe("/position_cmd", 1, &PositionControllerNode::posCmdCb, this);
        // pose_sub = nh.subscribe("/uav/state/pose", 1, &PositionControllerNode::poseCb, this);
        pose_sub = nh.subscribe("/airsim_node/drone_1/debug/pose_gt", 1, &PositionControllerNode::poseCb, this);
        vel_pub = nh.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_body_cmd", 1);

        const double timer_period = 1.0 / std::max(ctrl_rate_hz, 1.0);
        timer = nh.createTimer(ros::Duration(timer_period), &PositionControllerNode::controlLoop, this);
    }

private:
    struct AxisProfile {
        double kp = 0.0;
        double ki = 0.0;
        double kd = 0.0;
        double max_out = 0.0;
        double max_integral = 0.0;
    };

    const char* modeName(uint8_t flag) const {
        switch (flag) {
            case FLAG_BRAKE_FOR_TURN:
                return "BRAKE_FOR_TURN";
            case FLAG_ROTATE_FOR_TURN:
                return "ROTATE_FOR_TURN";
            case FLAG_TRACK:
            default:
                return "TRACK";
        }
    }

    static double clampWithLimit(double value, double limit) {
        return std::clamp(value, -limit, limit);
    }

    static void clampVectorNorm(double& x, double& y, double limit) {
        if (limit <= 1e-6) {
            x = 0.0;
            y = 0.0;
            return;
        }

        const double norm = std::sqrt(x * x + y * y);
        if (norm <= limit) {
            return;
        }

        const double scale = limit / norm;
        x *= scale;
        y *= scale;
    }

    static void clampVectorNorm3(double& x, double& y, double& z, double limit) {
        if (limit <= 1e-6) {
            x = 0.0;
            y = 0.0;
            z = 0.0;
            return;
        }

        const double norm = std::sqrt(x * x + y * y + z * z);
        if (norm <= limit) {
            return;
        }

        const double scale = limit / norm;
        x *= scale;
        y *= scale;
        z *= scale;
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void publishStop(bool stop_flag) {
        airsim_ros::VelCmd cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.vx = 0.0;
        cmd.vy = 0.0;
        cmd.vz = 0.0;
        cmd.yawRate = 0.0;
        cmd.va = static_cast<uint8_t>(xy_accel_limit);
        cmd.stop = stop_flag ? 1 : 0;
        vel_pub.publish(cmd);
    }

    void loadParams() {
        // XY 轴 (一致)
        track_xy.kp = nh.param("kp_xy", 1.0);
        track_xy.ki = nh.param("ki_xy", 0.0);
        track_xy.kd = nh.param("kd_xy", 0.05);
        track_xy.max_out = nh.param("max_vel_xy", 3.0);
        track_xy.max_integral = nh.param("max_integral_xy", 1.0);

        // Z 轴 (独立)
        track_z.kp = nh.param("kp_z", 1.5);
        track_z.ki = nh.param("ki_z", 0.01);
        track_z.kd = nh.param("kd_z", 0.1);
        track_z.max_out = nh.param("max_vel_z", 1.5);
        track_z.max_integral = nh.param("max_integral_z", 1.0);

        startup_xy.kp = nh.param("startup_kp_xy", 0.6);
        startup_xy.ki = nh.param("startup_ki_xy", track_xy.ki);
        startup_xy.kd = nh.param("startup_kd_xy", track_xy.kd);
        startup_xy.max_out = nh.param("startup_max_vel_xy", 3.0);
        startup_xy.max_integral = nh.param("startup_max_integral_xy", track_xy.max_integral);

        startup_z.kp = nh.param("startup_kp_z", 1.2);
        startup_z.ki = nh.param("startup_ki_z", track_z.ki);
        startup_z.kd = nh.param("startup_kd_z", track_z.kd);
        startup_z.max_out = nh.param("startup_max_vel_z", 1.5);
        startup_z.max_integral = nh.param("startup_max_integral_z", track_z.max_integral);

        ctrl_rate_hz = nh.param("ctrl_rate_hz", 50.0);
        cmd_timeout = nh.param("cmd_timeout", 0.3);
        xy_accel_limit = nh.param("xy_accel_limit", 4);
        max_yaw_rate = nh.param("max_yaw_rate", 1.0);
        rotate_max_yaw_rate = nh.param("rotate_max_yaw_rate", 0.7);
        yaw_kp_track = nh.param("yaw_kp_track", 2.5);
        yaw_kp_rotate = nh.param("yaw_kp_rotate", 0.9);
        yaw_rate_cmd_scale = nh.param("yaw_rate_cmd_scale", 57.29577951308232);
        startup_profile_hold_dist = nh.param("startup_profile_hold_dist", 40.0);
        startup_profile_blend_dist = nh.param("startup_profile_blend_dist", 30.0);
        acc_ff_gain_xy = nh.param("acc_ff_gain_xy", 0.0);
        acc_ff_gain_z = nh.param("acc_ff_gain_z", 0.0);
        max_acc_ff_xy = nh.param("max_acc_ff_xy", 0.0);
        max_acc_ff_z = nh.param("max_acc_ff_z", 0.0);
        max_acc_ff_total = nh.param("max_acc_ff_total", 0.0);
        // 该仿真里 PositionCommand 和 /uav/state/pose 默认都按 NED 提供。
        // 控制器内部统一把 z 转成 z-up，再直接发送给 VelCmd.vz。
        pos_cmd_z_sign = nh.param("position_cmd_z_sign", -1.0);
        pose_z_sign = nh.param("pose_z_sign", -1.0);

        applyAxisProfile(track_xy, track_z);
    }

    void posCmdCb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        target_pos = *msg;
        last_cmd_time = ros::Time::now();
        received_cmd = true;
        if (!startup_window_started) {
            startup_window_started = true;
            startup_window_begin = last_cmd_time;
        }
    }

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        curr_pose = *msg;
        curr_yaw = tf::getYaw(msg->pose.orientation);
        received_pose = true;
        if (!startup_origin_captured && startup_window_started) {
            startup_origin_pose = *msg;
            startup_origin_captured = true;
        }
    }

    void controlLoop(const ros::TimerEvent&) {
        if (!received_cmd || !received_pose) return;

        const ros::Time now = ros::Time::now();
        double dt = (now - last_control_time).toSec();
        last_control_time = now;
        dt = std::clamp(dt, 0.001, 0.1);

        if ((now - last_cmd_time).toSec() > cmd_timeout) {
            pid_x.reset();
            pid_y.reset();
            pid_z.reset();
            profile_initialized = false;
            startup_origin_captured = false;
            publishStop(false);
            ROS_WARN_THROTTLE(1.0, "Position command timeout, publishing zero velocity.");
            return;
        }

        updateControlProfile(now);

        if (target_pos.trajectory_flag == FLAG_BRAKE_FOR_TURN) {
            pid_x.reset();
            pid_y.reset();
            pid_z.reset();
            ROS_INFO_THROTTLE(0.5, "PosController mode=%s -> publishStop(true)", modeName(target_pos.trajectory_flag));
            publishStop(true);
            return;
        }

        const double curr_x = curr_pose.pose.position.x;
        const double curr_y = curr_pose.pose.position.y;
        const double curr_z = pose_z_sign * curr_pose.pose.position.z;

        const double target_x = target_pos.position.x;
        const double target_y = target_pos.position.y;
        const double target_z = pos_cmd_z_sign * target_pos.position.z;

        double err_x = target_x - curr_x;
        double err_y = target_y - curr_y;
        double err_z = target_z - curr_z;

        // 2. 打印误差 (每 0.5 秒打印一次，避免刷屏)
        // ROS_INFO_THROTTLE(0.5,
        //                   "Pos Error [ctrl frame] -> X: %.3f, Y: %.3f, Z: %.3f | target_z: %.3f curr_z: %.3f",
        //                   err_x, err_y, err_z, target_z, curr_z);

        // 1. 世界系下的位置 PID
        double v_pid_x = pid_x.compute(target_x, curr_x, dt);
        double v_pid_y = pid_y.compute(target_y, curr_y, dt);
        double v_pid_z = pid_z.compute(target_z, curr_z, dt);

        v_pid_x = clampWithLimit(v_pid_x, pid_x.max_out);
        v_pid_y = clampWithLimit(v_pid_y, pid_y.max_out);
        v_pid_z = clampWithLimit(v_pid_z, pid_z.max_out);

        // Consume planner acceleration as a bounded velocity feedforward so the
        // DP network can bias the translational command without replacing the
        // existing position loop.
        double acc_ff_x = target_pos.acceleration.x;
        double acc_ff_y = target_pos.acceleration.y;
        double acc_ff_z = pos_cmd_z_sign * target_pos.acceleration.z;
        clampVectorNorm(acc_ff_x, acc_ff_y, max_acc_ff_xy);
        acc_ff_z = clampWithLimit(acc_ff_z, max_acc_ff_z);
        clampVectorNorm3(acc_ff_x, acc_ff_y, acc_ff_z, max_acc_ff_total);

        const double v_ff_x = acc_ff_gain_xy * acc_ff_x * dt;
        const double v_ff_y = acc_ff_gain_xy * acc_ff_y * dt;
        const double v_ff_z = acc_ff_gain_z * acc_ff_z * dt;

        double v_w_x = v_pid_x + v_ff_x;
        double v_w_y = v_pid_y + v_ff_y;
        double v_w_z = v_pid_z + v_ff_z;

        v_w_x = clampWithLimit(v_w_x, pid_x.max_out);
        v_w_y = clampWithLimit(v_w_y, pid_y.max_out);
        v_w_z = clampWithLimit(v_w_z, pid_z.max_out);

        // Path follower still publishes desired speed. We no longer add it as
        // feedforward, but we do respect it as a hard speed cap so the position
        // loop cannot outrun the intended path-tracking speed.
        const double vel_cap_xy =
            std::sqrt(target_pos.velocity.x * target_pos.velocity.x +
                      target_pos.velocity.y * target_pos.velocity.y);
        clampVectorNorm(v_w_x, v_w_y, std::min(pid_x.max_out, vel_cap_xy));

        const double planner_vz_ref = std::fabs(target_pos.velocity.z);

        if (target_pos.trajectory_flag == FLAG_ROTATE_FOR_TURN) {
            v_w_x = 0.0;
            v_w_y = 0.0;
            v_w_z = 0.0;
        }

        ROS_INFO_THROTTLE(0.5,
                          "PosController mode=%s target_z=%.2f curr_z=%.2f err_z=%.2f vel_pid=(%.2f,%.2f,%.2f) vel_ff=(%.2f,%.2f,%.2f) vel_cmd=(%.2f,%.2f,%.2f) acc_ff=(%.2f,%.2f,%.2f) vel_cap=(%.2f,%.2f) yaw=%.2f yaw_dot=%.2f",
                          modeName(target_pos.trajectory_flag), target_z, curr_z, err_z,
                          v_pid_x, v_pid_y, v_pid_z,
                          v_ff_x, v_ff_y, v_ff_z,
                          v_w_x, v_w_y, v_w_z,
                          acc_ff_x, acc_ff_y, acc_ff_z,
                          vel_cap_xy, planner_vz_ref,
                          target_pos.yaw, target_pos.yaw_dot);

        // 2. 坐标变换 World -> Body
        double cos_y = std::cos(curr_yaw);
        double sin_y = std::sin(curr_yaw);

        // 3. 构造修正后的 VelCmd (对应你提供的图片结构)
        airsim_ros::VelCmd cmd;
        cmd.header.stamp = now;
        cmd.vx =  v_w_x * cos_y + v_w_y * sin_y;
        cmd.vy = -v_w_x * sin_y + v_w_y * cos_y;
        cmd.vz =  v_w_z;
        const double yaw_err = normalizeAngle(target_pos.yaw - curr_yaw);
        const bool rotate_mode = (target_pos.trajectory_flag == FLAG_ROTATE_FOR_TURN);
        const double yaw_kp = rotate_mode ? yaw_kp_rotate : yaw_kp_track;
        const double yaw_rate_limit = rotate_mode ? rotate_max_yaw_rate : max_yaw_rate;
        const double yaw_p_term = yaw_kp * yaw_err;
        const double yaw_rate_raw = target_pos.yaw_dot + yaw_p_term;
        const double yaw_rate_limited = clampWithLimit(yaw_rate_raw, yaw_rate_limit);
        cmd.yawRate = yaw_rate_cmd_scale * yaw_rate_limited;
        // ROS_INFO_THROTTLE(0.5,
        //                   "Yaw Ctrl -> target_yaw: %.3f curr_yaw: %.3f err: %.3f yaw_dot_ff: %.3f p_term: %.3f raw_rad: %.3f out_rad: %.3f pub: %.3f",
        //                   target_pos.yaw, curr_yaw, yaw_err, target_pos.yaw_dot, yaw_p_term,
        //                   yaw_rate_raw, yaw_rate_limited, cmd.yawRate);

        cmd.va = static_cast<uint8_t>(std::clamp(xy_accel_limit, 0, 255));
        cmd.stop = 0;

        vel_pub.publish(cmd);
    }

    void applyAxisProfile(const AxisProfile& xy, const AxisProfile& z) {
        pid_x.kp = pid_y.kp = xy.kp;
        pid_x.ki = pid_y.ki = xy.ki;
        pid_x.kd = pid_y.kd = xy.kd;
        pid_x.max_out = pid_y.max_out = xy.max_out;
        pid_x.max_integral = pid_y.max_integral = xy.max_integral;

        pid_z.kp = z.kp;
        pid_z.ki = z.ki;
        pid_z.kd = z.kd;
        pid_z.max_out = z.max_out;
        pid_z.max_integral = z.max_integral;
    }

    AxisProfile blendProfile(const AxisProfile& a, const AxisProfile& b, double alpha) const {
        AxisProfile out;
        out.kp = a.kp + (b.kp - a.kp) * alpha;
        out.ki = a.ki + (b.ki - a.ki) * alpha;
        out.kd = a.kd + (b.kd - a.kd) * alpha;
        out.max_out = a.max_out + (b.max_out - a.max_out) * alpha;
        out.max_integral = a.max_integral + (b.max_integral - a.max_integral) * alpha;
        return out;
    }

    void updateControlProfile(const ros::Time& now) {
        double blend_alpha = 0.0;
        if (startup_origin_captured && startup_profile_blend_dist >= 0.0) {
            const double dx = curr_pose.pose.position.x - startup_origin_pose.pose.position.x;
            const double dy = curr_pose.pose.position.y - startup_origin_pose.pose.position.y;
            const double traveled = std::sqrt(dx * dx + dy * dy);
            if (startup_profile_blend_dist <= 1e-3) {
                blend_alpha = traveled >= startup_profile_hold_dist ? 1.0 : 0.0;
            } else {
                blend_alpha = std::clamp(
                    (traveled - startup_profile_hold_dist) / startup_profile_blend_dist,
                    0.0, 1.0);
            }
            ROS_INFO_THROTTLE(0.5,
                              "PosController profile blend traveled=%.2f hold=%.2f blend=%.2f alpha=%.2f",
                              traveled, startup_profile_hold_dist, startup_profile_blend_dist, blend_alpha);
        }

        if (profile_initialized && std::fabs(blend_alpha - last_profile_alpha) < 1e-3) {
            return;
        }

        profile_initialized = true;
        last_profile_alpha = blend_alpha;
        startup_profile_active = blend_alpha < 0.999;

        const AxisProfile blended_xy = blendProfile(startup_xy, track_xy, blend_alpha);
        const AxisProfile blended_z = blendProfile(startup_z, track_z, blend_alpha);
        applyAxisProfile(blended_xy, blended_z);

        if (!profile_log_initialized || std::fabs(blend_alpha - last_profile_log_alpha) > 0.2 || blend_alpha >= 0.999) {
            profile_log_initialized = true;
            last_profile_log_alpha = blend_alpha;
            ROS_INFO("PosController blended profile alpha=%.2f xy(kp=%.2f,max=%.2f) z(kp=%.2f,max=%.2f)",
                     blend_alpha, blended_xy.kp, blended_xy.max_out, blended_z.kp, blended_z.max_out);
        }
    }

    ros::NodeHandle nh;
    ros::Subscriber pos_cmd_sub, pose_sub;
    ros::Publisher vel_pub;
    ros::Timer timer;

    PID pid_x, pid_y, pid_z;
    AxisProfile track_xy, track_z;
    AxisProfile startup_xy, startup_z;
    quadrotor_msgs::PositionCommand target_pos;
    geometry_msgs::PoseStamped curr_pose;
    geometry_msgs::PoseStamped startup_origin_pose;
    double curr_yaw = 0.0;
    double ctrl_rate_hz = 50.0;
    double cmd_timeout = 0.3;
    int xy_accel_limit = 4;
    double max_yaw_rate = 1.0;
    double rotate_max_yaw_rate = 0.7;
    double yaw_kp_track = 2.5;
    double yaw_kp_rotate = 0.9;
    double yaw_rate_cmd_scale = 57.29577951308232;
    double startup_profile_hold_dist = 40.0;
    double startup_profile_blend_dist = 30.0;
    double acc_ff_gain_xy = 0.0;
    double acc_ff_gain_z = 0.0;
    double max_acc_ff_xy = 0.0;
    double max_acc_ff_z = 0.0;
    double max_acc_ff_total = 0.0;
    double pos_cmd_z_sign = 1.0;
    double pose_z_sign = -1.0;
    ros::Time last_cmd_time;
    ros::Time last_control_time;
    ros::Time startup_window_begin;
    bool received_cmd = false, received_pose = false;
    bool startup_window_started = false;
    bool startup_origin_captured = false;
    bool startup_profile_active = false;
    bool profile_initialized = false;
    bool profile_log_initialized = false;
    double last_profile_alpha = -1.0;
    double last_profile_log_alpha = -1.0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pos_controller_node");
    PositionControllerNode node;
    ros::spin();
    return 0;
}
