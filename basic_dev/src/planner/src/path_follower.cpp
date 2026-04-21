#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <algorithm>
#include <vector>

class PathFollower {
public:
    static constexpr uint8_t FLAG_TRACK = 0;
    enum class TurnState { NORMAL = 0, EXIT = 3 };

    PathFollower() : nh("~") {
        std::string cmd_topic;
        // 加载参数
        nh.param("look_ahead_distance", look_ahead_dist, 2.0); // 预瞄距离，越大越平滑但切圆越严重
        nh.param("min_dist_threshold", min_dist_th, 0.2);     // 距离死区，防止原地抖动
        nh.param("cruise_speed", cruise_speed, 4.0);          // 路径跟随速度前馈
        nh.param("slow_down_radius", slow_down_radius, 1.0);  // 终点附近减速半径
        nh.param("max_climb_speed", max_climb_speed, 3.0);    // 上下坡时的 z 轴前馈限幅
        nh.param("z_feedforward_gain", z_feedforward_gain, 1.5); // z 轴前馈增益
        nh.param("forward_search_window", forward_search_window, 80); // 按 CSV 行号向前搜索，避免跳到几何上更近的回程路径
        nh.param("turn_slow_angle_deg", turn_slow_angle_deg, 25.0);
        nh.param("turn_sharp_angle_deg", turn_sharp_angle_deg, 70.0);
        nh.param("turn_min_speed_ratio", turn_min_speed_ratio, 0.45);
        nh.param("turn_min_look_ahead", turn_min_look_ahead, 1.5);
        nh.param("enable_turnaround_mode", enable_turnaround_mode, true);
        nh.param("turnaround_angle_deg", turnaround_angle_deg, 135.0);
        nh.param("turnaround_search_ahead", turnaround_search_ahead, 160);
        nh.param("turnaround_activate_dist", turnaround_activate_dist, 10.0);
        nh.param("turnaround_look_ahead", turnaround_look_ahead, 1.0);
        nh.param("turnaround_speed", turnaround_speed, 1.5);
        nh.param("turnaround_exit_points", turnaround_exit_points, 30);
        nh.param("enable_fixed_turnaround_points", enable_fixed_turnaround_points, true);
        nh.param("fixed_turnaround_activate_dist", fixed_turnaround_activate_dist, 22.0);
        nh.param("fixed_turnaround_trigger_ahead_points", fixed_turnaround_trigger_ahead_points, 80);
        nh.param("turnaround_z_step_limit", turnaround_z_step_limit, 0.03);
        nh.param("enable_post_turn_yaw_align", enable_post_turn_yaw_align, true);
        nh.param("post_turn_yaw_align_deg", post_turn_yaw_align_deg, 12.0);
        nh.param("startup_ramp_duration", startup_ramp_duration, 4.0);
        nh.param("startup_speed_ratio", startup_speed_ratio, 0.20);
        nh.param("startup_look_ahead_ratio", startup_look_ahead_ratio, 0.20);
        nh.param("startup_min_look_ahead", startup_min_look_ahead, 0.6);
        nh.param("enable_vertical_catchup", enable_vertical_catchup, true);
        nh.param("vertical_catchup_activate_z_err", vertical_catchup_activate_z_err, 2.0);
        nh.param("vertical_catchup_release_z_err", vertical_catchup_release_z_err, 1.0);
        nh.param("vertical_catchup_look_ahead", vertical_catchup_look_ahead, 0.8);
        nh.param("vertical_catchup_speed_ratio", vertical_catchup_speed_ratio, 0.45);
        nh.param("debug_log_active_points", debug_log_active_points, true);
        nh.param<std::string>("cmd_topic", cmd_topic, "/position_cmd");

        path_sub = nh.subscribe("/drone_1/saved_path", 1, &PathFollower::pathCb, this);
        // 源工程这里订阅 /uav/state/pose；当前工程继续使用 AirSim 真值位姿以保持现有链路可运行。
        pose_sub = nh.subscribe("/airsim_node/drone_1/debug/pose_gt", 1, &PathFollower::poseCb, this);
        cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>(cmd_topic, 1);

        timer = nh.createTimer(ros::Duration(0.02), &PathFollower::controlLoop, this); // 50Hz
        ROS_INFO("Path Follower with Yaw control initialized. cmd_topic=%s", cmd_topic.c_str());
    }

private:
    struct FixedTurnaroundPoint {
        double x;
        double y;
        int path_idx = -1;
        int trigger_idx = -1;
    };

    const char* modeName(uint8_t flag) const {
        switch (flag) {
            case FLAG_TRACK:
            default:
                return "TRACK";
        }
    }

    const char* turnStateName(TurnState state) const {
        switch (state) {
            case TurnState::EXIT:
                return "EXIT";
            case TurnState::NORMAL:
            default:
                return "NORMAL";
        }
    }

    void pathCb(const nav_msgs::Path::ConstPtr& msg) {
        const bool path_was_empty = !has_path || current_path.poses.empty();
        const bool path_shape_changed =
            current_path.poses.size() != msg->poses.size();
        current_path = *msg;
        has_path = true;
        if (enable_fixed_turnaround_points) {
            refreshFixedTurnaroundIndices();
        }
        if (path_was_empty || path_shape_changed) {
            progress_idx = -1;
        }
        ROS_INFO("PathFollower received path: poses=%zu shape_changed=%s", current_path.poses.size(), path_shape_changed ? "YES" : "NO");
    }

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        const ros::Time now = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        if (has_pose) {
            const double dt = std::max((now - last_pose_time).toSec(), 1e-3);
            curr_speed_xy = getHorizontalDist(curr_pose.pose.position, msg->pose.position) / dt;
        }
        curr_pose = *msg;
        curr_yaw = tf::getYaw(msg->pose.orientation);
        last_pose_time = now;
        has_pose = true;
    }

    double getDist(geometry_msgs::Point p1, geometry_msgs::Point p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
    }

    double getHorizontalDist(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const {
        const double dx = p1.x - p2.x;
        const double dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double normalizeAngle(double angle) const {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double computeTargetYaw(const geometry_msgs::Point& target) const {
        const double dx = target.x - curr_pose.pose.position.x;
        const double dy = target.y - curr_pose.pose.position.y;
        if (std::sqrt(dx * dx + dy * dy) > 1e-3) {
            return std::atan2(dy, dx);
        }
        return curr_yaw;
    }

    double limitZStep(double target_z) {
        if (!has_last_cmd_z) {
            last_cmd_z = target_z;
            has_last_cmd_z = true;
            return target_z;
        }

        const double dz = target_z - last_cmd_z;
        const double limited = last_cmd_z + std::clamp(dz, -turnaround_z_step_limit, turnaround_z_step_limit);
        last_cmd_z = limited;
        return limited;
    }

    double computeTurnAngleDeg(int idx) const {
        if (idx <= 0 || idx >= static_cast<int>(current_path.poses.size()) - 1) {
            return 0.0;
        }

        const auto& prev = current_path.poses[idx - 1].pose.position;
        const auto& curr = current_path.poses[idx].pose.position;
        const auto& next = current_path.poses[idx + 1].pose.position;

        const double v1x = curr.x - prev.x;
        const double v1y = curr.y - prev.y;
        const double v2x = next.x - curr.x;
        const double v2y = next.y - curr.y;
        const double n1 = std::sqrt(v1x * v1x + v1y * v1y);
        const double n2 = std::sqrt(v2x * v2x + v2y * v2y);
        if (n1 < 1e-3 || n2 < 1e-3) {
            return 0.0;
        }

        double cos_angle = (v1x * v2x + v1y * v2y) / (n1 * n2);
        cos_angle = std::clamp(cos_angle, -1.0, 1.0);
        return std::acos(cos_angle) * 180.0 / M_PI;
    }

    bool findUpcomingTurnaround(int nearest_idx, int& turnaround_idx, double& dist_to_turnaround) const {
        turnaround_idx = -1;
        dist_to_turnaround = 1e9;
        if (!enable_turnaround_mode || current_path.poses.size() < 3) {
            return false;
        }

        if (enable_fixed_turnaround_points &&
            findFixedTurnaround(nearest_idx, turnaround_idx, dist_to_turnaround)) {
            return true;
        }

        const int path_size = static_cast<int>(current_path.poses.size());
        const int search_end = std::min(nearest_idx + turnaround_search_ahead, path_size - 2);
        double best_angle = turnaround_angle_deg;

        for (int i = std::max(1, nearest_idx); i <= search_end; ++i) {
            const double angle_deg = computeTurnAngleDeg(i);
            if (angle_deg < best_angle) {
                continue;
            }

            const double d = getHorizontalDist(curr_pose.pose.position, current_path.poses[i].pose.position);
            if (angle_deg > best_angle || d < dist_to_turnaround) {
                best_angle = angle_deg;
                turnaround_idx = i;
                dist_to_turnaround = d;
            }
        }

        return turnaround_idx >= 0 && dist_to_turnaround <= turnaround_activate_dist;
    }

    bool findFixedTurnaround(int nearest_idx, int& turnaround_idx, double& dist_to_turnaround) const {
        if (fixed_turnaround_points.empty() || current_path.poses.empty()) {
            return false;
        }

        const int path_size = static_cast<int>(current_path.poses.size());
        int best_idx = -1;
        double best_dist = 1e9;

        for (const auto& point : fixed_turnaround_points) {
            if (point.path_idx < 0 || point.path_idx >= path_size ||
                point.trigger_idx < 0 || point.trigger_idx >= path_size) {
                continue;
            }

            // Only consider fixed turnaround points that are still ahead on the route.
            if (point.path_idx + turnaround_exit_points < nearest_idx) {
                continue;
            }

            const geometry_msgs::Point& trigger_pt = current_path.poses[point.trigger_idx].pose.position;
            const double d = getHorizontalDist(curr_pose.pose.position, trigger_pt);
            if (d <= fixed_turnaround_activate_dist && d < best_dist) {
                best_dist = d;
                best_idx = point.path_idx;
            }
        }

        if (best_idx < 0) {
            return false;
        }

        turnaround_idx = best_idx;
        dist_to_turnaround = best_dist;
        return true;
    }

    void refreshFixedTurnaroundIndices() {
        if (current_path.poses.empty()) {
            return;
        }

        if (fixed_turnaround_points.empty()) {
            fixed_turnaround_points = {
                {546.482, 521.007, -1},
                {1170.496, -423.165, -1},
                {714.613, -722.970, -1},
                {652.462, -172.698, -1},
                {267.311, 393.279, -1},
                {1321.899, 151.379, -1},
            };
        }

        for (auto& point : fixed_turnaround_points) {
            int best_idx = -1;
            double best_dist = 1e9;
            for (int i = 0; i < static_cast<int>(current_path.poses.size()); ++i) {
                const auto& p = current_path.poses[i].pose.position;
                const double dx = p.x - point.x;
                const double dy = p.y - point.y;
                const double d = std::sqrt(dx * dx + dy * dy);
                if (d < best_dist) {
                    best_dist = d;
                    best_idx = i;
                }
            }
            point.path_idx = best_idx;
            point.trigger_idx =
                best_idx >= 0 ? std::max(0, best_idx - fixed_turnaround_trigger_ahead_points) : -1;
        }
    }

    double computeUpcomingTurnAngleDeg(int nearest_idx, double preview_distance) const {
        if (current_path.poses.size() < 3) {
            return 0.0;
        }

        const int path_size = static_cast<int>(current_path.poses.size());
        const int start_idx = std::max(1, nearest_idx);
        const int end_idx = std::min(path_size - 2, start_idx + forward_search_window);

        double accum_d = 0.0;
        double max_angle_deg = 0.0;
        for (int i = start_idx; i <= end_idx; ++i) {
            max_angle_deg = std::max(max_angle_deg, computeTurnAngleDeg(i));
            if (i < path_size - 1) {
                accum_d += getHorizontalDist(current_path.poses[i].pose.position,
                                             current_path.poses[i + 1].pose.position);
            }
            if (accum_d >= preview_distance) {
                break;
            }
        }
        return max_angle_deg;
    }

    void getPathDirection(int target_idx, double& dir_x, double& dir_y, double& dir_z) const {
        dir_x = 0.0;
        dir_y = 0.0;
        dir_z = 0.0;

        if (current_path.poses.size() < 2) {
            return;
        }

        geometry_msgs::Point from;
        geometry_msgs::Point to;
        if (target_idx < static_cast<int>(current_path.poses.size()) - 1) {
            from = current_path.poses[target_idx].pose.position;
            to = current_path.poses[target_idx + 1].pose.position;
        } else {
            from = current_path.poses[target_idx - 1].pose.position;
            to = current_path.poses[target_idx].pose.position;
        }

        dir_x = to.x - from.x;
        dir_y = to.y - from.y;
        dir_z = to.z - from.z;

        const double norm = std::sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
        if (norm > 1e-3) {
            dir_x /= norm;
            dir_y /= norm;
            dir_z /= norm;
            return;
        }

        dir_x = current_path.poses[target_idx].pose.position.x - curr_pose.pose.position.x;
        dir_y = current_path.poses[target_idx].pose.position.y - curr_pose.pose.position.y;
        dir_z = current_path.poses[target_idx].pose.position.z - curr_pose.pose.position.z;
        const double fallback_norm = std::sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
        if (fallback_norm > 1e-3) {
            dir_x /= fallback_norm;
            dir_y /= fallback_norm;
            dir_z /= fallback_norm;
        } else {
            dir_x = 0.0;
            dir_y = 0.0;
            dir_z = 0.0;
        }
    }

    void controlLoop(const ros::TimerEvent&) {
        if (!has_path || !has_pose || current_path.poses.empty()) return;

        if (!startup_ramp_started) {
            startup_ramp_started = true;
            startup_ramp_begin = ros::Time::now();
        }

        const int path_size = static_cast<int>(current_path.poses.size());

        // 1. 首次进入路径时全局定位一次，之后只允许按 CSV 行号向前推进
        if (progress_idx < 0) {
            int init_idx = 0;
            double init_min_d = 1e6;
            for (int i = 0; i < path_size; ++i) {
                double d = getHorizontalDist(curr_pose.pose.position, current_path.poses[i].pose.position);
                if (d < init_min_d) {
                    init_min_d = d;
                    init_idx = i;
                }
            }
            progress_idx = init_idx;
        }

        int nearest_idx = progress_idx;
        double min_d = getHorizontalDist(curr_pose.pose.position, current_path.poses[progress_idx].pose.position);
        const int search_end = std::min(progress_idx + forward_search_window, path_size - 1);
        for (int i = progress_idx; i <= search_end; ++i) {
            double d = getHorizontalDist(curr_pose.pose.position, current_path.poses[i].pose.position);
            if (d < min_d) {
                min_d = d;
                nearest_idx = i;
            }
        }
        progress_idx = nearest_idx;

        const double local_path_z = current_path.poses[nearest_idx].pose.position.z;
        const double curr_z = curr_pose.pose.position.z;
        const double local_z_err = local_path_z - curr_z;

        if (enable_vertical_catchup) {
            if (vertical_catchup_active) {
                if (std::fabs(local_z_err) <= vertical_catchup_release_z_err) {
                    vertical_catchup_active = false;
                }
            } else if (std::fabs(local_z_err) >= vertical_catchup_activate_z_err) {
                vertical_catchup_active = true;
            }
        } else {
            vertical_catchup_active = false;
        }

        int turnaround_idx = -1;
        double dist_to_turnaround = 1e9;
        const bool turnaround_detected = findUpcomingTurnaround(nearest_idx, turnaround_idx, dist_to_turnaround);
        if (turn_state == TurnState::NORMAL && turnaround_detected) {
            turn_state = TurnState::EXIT;
            latched_turnaround_idx = turnaround_idx;
        }
        if (turn_state != TurnState::NORMAL) {
            turnaround_idx = latched_turnaround_idx;
            dist_to_turnaround = getHorizontalDist(curr_pose.pose.position, current_path.poses[turnaround_idx].pose.position);
            if (nearest_idx > turnaround_idx) {
                turn_state = TurnState::NORMAL;
                latched_turnaround_idx = -1;
                turnaround_idx = -1;
                dist_to_turnaround = 1e9;
                has_last_cmd_z = false;
                post_turn_yaw_align_pending = enable_post_turn_yaw_align;
            }
        }
        const bool turnaround_active = turn_state != TurnState::NORMAL || turnaround_detected;
        const bool turnaround_slow_phase =
            turnaround_active && turnaround_idx >= 0 && nearest_idx <= turnaround_idx;
        const double turn_angle_deg = computeUpcomingTurnAngleDeg(nearest_idx, look_ahead_dist * 1.5);
        const double turn_ratio = std::clamp(
            (turn_angle_deg - turn_slow_angle_deg) /
            std::max(turn_sharp_angle_deg - turn_slow_angle_deg, 1e-3),
            0.0, 1.0);
        const double adaptive_look_ahead = look_ahead_dist - turn_ratio * (look_ahead_dist - turn_min_look_ahead);
        double active_look_ahead = turnaround_slow_phase ? turnaround_look_ahead : adaptive_look_ahead;
        if (vertical_catchup_active && !turnaround_active) {
            active_look_ahead = std::min(active_look_ahead, vertical_catchup_look_ahead);
        }
        if (!turnaround_active && startup_ramp_duration > 1e-3) {
            const double elapsed = std::max((ros::Time::now() - startup_ramp_begin).toSec(), 0.0);
            const double ramp_alpha = std::clamp(elapsed / startup_ramp_duration, 0.0, 1.0);
            const double look_ahead_ratio = std::clamp(startup_look_ahead_ratio, 0.0, 1.0);
            const double startup_look_ahead_cap = std::max(
                startup_min_look_ahead,
                look_ahead_dist * (look_ahead_ratio + (1.0 - look_ahead_ratio) * ramp_alpha));
            active_look_ahead = std::min(active_look_ahead, startup_look_ahead_cap);
        }
        if (turnaround_active) {
            ROS_INFO_THROTTLE(0.5,
                              "Turnaround state=%d -> progress_idx: %d nearest_idx: %d turnaround_idx: %d dist: %.2f look_ahead: %.2f speed: %.2f vxy: %.2f",
                              static_cast<int>(turn_state), progress_idx, nearest_idx, turnaround_idx,
                              dist_to_turnaround, active_look_ahead,
                              turnaround_slow_phase ? turnaround_speed : cruise_speed, curr_speed_xy);
        }

        // 2. 提取预瞄点 (Look-ahead)
        int target_idx = nearest_idx;
        double accum_d = 0;
        for (int i = nearest_idx; i < path_size - 1; ++i) {
            accum_d += getHorizontalDist(current_path.poses[i].pose.position,
                                         current_path.poses[i+1].pose.position);
            target_idx = i + 1;
            if (turnaround_active && nearest_idx <= turnaround_idx) {
                target_idx = std::min(target_idx, turnaround_idx + turnaround_exit_points);
            }
            if (accum_d >= active_look_ahead) break;
        }
        target_idx = std::max(target_idx, nearest_idx);
        int yaw_target_idx = target_idx;
        if (turnaround_active && nearest_idx <= turnaround_idx) {
            yaw_target_idx = std::min(path_size - 1, turnaround_idx + turnaround_exit_points);
        }
        yaw_target_idx = std::max(yaw_target_idx, target_idx);

        // 3. 构建位置指令
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";
        cmd.position = current_path.poses[target_idx].pose.position;
        const geometry_msgs::Point turnaround_yaw_target = current_path.poses[yaw_target_idx].pose.position;
        cmd.trajectory_flag = FLAG_TRACK;
        if (turnaround_active || vertical_catchup_active) {
            // During turn handling, follow the nearby path altitude instead of the
            // far look-ahead point altitude to avoid large z jumps.
            cmd.position.z = local_path_z;
        }

        if (turnaround_slow_phase) {
            cmd.position.z = limitZStep(cmd.position.z);
        } else {
            last_cmd_z = cmd.position.z;
            has_last_cmd_z = true;
        }

        // 4. Yaw 仍然跟当前目标点
        double dx = cmd.position.x - curr_pose.pose.position.x;
        double dy = cmd.position.y - curr_pose.pose.position.y;
        double dist_to_target = std::sqrt(dx * dx + dy * dy);
        const bool is_last_point = (target_idx == static_cast<int>(current_path.poses.size()) - 1);

        if (turnaround_slow_phase) {
            cmd.yaw = computeTargetYaw(turnaround_yaw_target);
        } else if (dist_to_target > min_dist_th) {
            cmd.yaw = computeTargetYaw(cmd.position);
        } else {
            cmd.yaw = curr_yaw;
        }

        bool post_turn_yaw_hold = false;
        if (post_turn_yaw_align_pending && !turnaround_slow_phase) {
            const double yaw_err = std::fabs(normalizeAngle(cmd.yaw - curr_yaw));
            if (yaw_err > post_turn_yaw_align_deg * M_PI / 180.0) {
                post_turn_yaw_hold = true;
            } else {
                post_turn_yaw_align_pending = false;
            }
        }

        double target_speed = cruise_speed;
        const double turn_speed = cruise_speed * (1.0 - turn_ratio * (1.0 - turn_min_speed_ratio));
        target_speed = std::min(target_speed, turn_speed);
        if (turnaround_slow_phase) {
            target_speed = std::min(target_speed, turnaround_speed);
        } else if (post_turn_yaw_hold) {
            target_speed = 0.0;
        }
        if (vertical_catchup_active && !turnaround_active) {
            target_speed = std::min(target_speed, cruise_speed * vertical_catchup_speed_ratio);
        }
        if (is_last_point && slow_down_radius > 1e-3) {
            const double dist_to_goal = getHorizontalDist(curr_pose.pose.position,
                                                          current_path.poses.back().pose.position);
            target_speed *= std::clamp(dist_to_goal / slow_down_radius, 0.0, 1.0);
        }

        if (startup_ramp_duration > 1e-3) {
            const double elapsed = std::max((ros::Time::now() - startup_ramp_begin).toSec(), 0.0);
            const double startup_ratio = std::clamp(startup_speed_ratio, 0.0, 1.0);
            const double ramp_alpha = std::clamp(elapsed / startup_ramp_duration, 0.0, 1.0);
            const double speed_cap = cruise_speed * (startup_ratio + (1.0 - startup_ratio) * ramp_alpha);
            target_speed = std::min(target_speed, speed_cap);
        }

        double dir_x = 0.0;
        double dir_y = 0.0;
        double dir_z = 0.0;
        getPathDirection(target_idx, dir_x, dir_y, dir_z);

        const double active_z_gain = turnaround_active ? 0.0 : z_feedforward_gain;
        const double xy_speed_cmd = target_speed;
        const double z_speed_ref = post_turn_yaw_hold ? cruise_speed : target_speed;
        cmd.velocity.x = xy_speed_cmd * dir_x;
        cmd.velocity.y = xy_speed_cmd * dir_y;
        cmd.velocity.z = turnaround_active
                             ? 0.0
                             : std::clamp(z_speed_ref * dir_z * active_z_gain,
                                          -max_climb_speed, max_climb_speed);
        if (post_turn_yaw_hold) {
            cmd.position.x = curr_pose.pose.position.x;
            cmd.position.y = curr_pose.pose.position.y;
            cmd.velocity.x = 0.0;
            cmd.velocity.y = 0.0;
        }
        cmd.yaw_dot = 0;

        ROS_INFO_THROTTLE(0.5,
                          "PathFollower mode=%s turn_detected=%s turn_state=%s vertical_catchup=%s z_err=%.2f nearest=%d target=%d turn_idx=%d dist_turn=%.2f speed=%.2f pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f)",
                          modeName(cmd.trajectory_flag), turnaround_active ? "YES" : "NO",
                          turnStateName(turn_state), vertical_catchup_active ? "YES" : "NO", local_z_err,
                          nearest_idx, target_idx, turnaround_idx, dist_to_turnaround, target_speed,
                          cmd.position.x, cmd.position.y, cmd.position.z,
                          cmd.velocity.x, cmd.velocity.y, cmd.velocity.z);
        if (debug_log_active_points) {
            const auto& nearest_pt = current_path.poses[nearest_idx].pose.position;
            const auto& target_pt = current_path.poses[target_idx].pose.position;
            const auto& yaw_target_pt = current_path.poses[yaw_target_idx].pose.position;
            ROS_INFO_THROTTLE(0.2,
                              "ActivePathPoints progress=%d/%d nearest_idx=%d pt=(%.2f,%.2f,%.2f) target_idx=%d pt=(%.2f,%.2f,%.2f) yaw_target_idx=%d pt=(%.2f,%.2f,%.2f)",
                              progress_idx, path_size - 1,
                              nearest_idx, nearest_pt.x, nearest_pt.y, nearest_pt.z,
                              target_idx, target_pt.x, target_pt.y, target_pt.z,
                              yaw_target_idx, yaw_target_pt.x, yaw_target_pt.y, yaw_target_pt.z);
        }

        cmd_pub.publish(cmd);
    }

    ros::NodeHandle nh;
    ros::Subscriber path_sub, pose_sub;
    ros::Publisher cmd_pub;
    ros::Timer timer;

    nav_msgs::Path current_path;
    geometry_msgs::PoseStamped curr_pose;
    double curr_yaw = 0.0;
    bool has_path = false, has_pose = false;
    double look_ahead_dist, min_dist_th;
    double cruise_speed = 4.0;
    double slow_down_radius = 1.0;
    double max_climb_speed = 3.0;
    double z_feedforward_gain = 1.5;
    int forward_search_window = 80;
    double turn_slow_angle_deg = 25.0;
    double turn_sharp_angle_deg = 70.0;
    double turn_min_speed_ratio = 0.45;
    double turn_min_look_ahead = 1.5;
    int progress_idx = -1;
    bool enable_turnaround_mode = true;
    double turnaround_angle_deg = 135.0;
    int turnaround_search_ahead = 160;
    double turnaround_activate_dist = 10.0;
    double turnaround_look_ahead = 1.0;
    double turnaround_speed = 1.5;
    int turnaround_exit_points = 30;
    bool enable_fixed_turnaround_points = true;
    double fixed_turnaround_activate_dist = 22.0;
    int fixed_turnaround_trigger_ahead_points = 80;
    double turnaround_z_step_limit = 0.03;
    bool enable_post_turn_yaw_align = true;
    double post_turn_yaw_align_deg = 12.0;
    bool debug_log_active_points = true;
    std::vector<FixedTurnaroundPoint> fixed_turnaround_points;
    TurnState turn_state = TurnState::NORMAL;
    int latched_turnaround_idx = -1;
    bool post_turn_yaw_align_pending = false;
    double curr_speed_xy = 0.0;
    double last_cmd_z = 0.0;
    bool has_last_cmd_z = false;
    ros::Time last_pose_time;
    double startup_ramp_duration = 4.0;
    double startup_speed_ratio = 0.20;
    double startup_look_ahead_ratio = 0.20;
    double startup_min_look_ahead = 0.6;
    bool enable_vertical_catchup = true;
    double vertical_catchup_activate_z_err = 2.0;
    double vertical_catchup_release_z_err = 1.0;
    double vertical_catchup_look_ahead = 0.8;
    double vertical_catchup_speed_ratio = 0.45;
    bool vertical_catchup_active = false;
    bool startup_ramp_started = false;
    ros::Time startup_ramp_begin;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower");
    PathFollower pf;
    ros::spin();
    return 0;
}
