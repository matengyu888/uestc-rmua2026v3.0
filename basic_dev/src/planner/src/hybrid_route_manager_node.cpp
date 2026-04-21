#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

struct RoutePoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  std::string segment;
  double global_order = 0.0;
};

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double spatialDistance(const RoutePoint& a, const RoutePoint& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double horizontalDistance(const RoutePoint& a, const RoutePoint& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

RoutePoint addScaled(const RoutePoint& point, double dx, double dy, double dz, double scale) {
  RoutePoint out = point;
  out.x += dx * scale;
  out.y += dy * scale;
  out.z += dz * scale;
  return out;
}

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> cells;
  std::string cell;
  bool in_quotes = false;

  for (char ch : line) {
    if (ch == '"') {
      in_quotes = !in_quotes;
      continue;
    }
    if (ch == ',' && !in_quotes) {
      cells.push_back(cell);
      cell.clear();
      continue;
    }
    cell.push_back(ch);
  }
  cells.push_back(cell);
  return cells;
}

std::string trim(const std::string& input) {
  const auto begin = input.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(begin, end - begin + 1);
}

geometry_msgs::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::Quaternion q;
  q.w = std::cos(0.5 * yaw);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  return q;
}

}  // namespace

class HybridRouteManager {
 public:
  static constexpr uint8_t FLAG_TRACK = 0;
  static constexpr uint8_t FLAG_BRAKE_FOR_TURN = 101;
  static constexpr uint8_t FLAG_ROTATE_FOR_TURN = 102;

  HybridRouteManager() : nh_(), pnh_("~") {
    loadParams();
    loadRouteFromCsv();
    densifyRoute();
    buildPathMessages();

    pose_sub_ = nh_.subscribe(pose_topic_, 1, &HybridRouteManager::poseCb, this);
    dp_cmd_sub_ = nh_.subscribe(dp_cmd_topic_, 1, &HybridRouteManager::dpCmdCb, this);

    cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(output_cmd_topic_, 1);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1);
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
    dense_path_pub_ = nh_.advertise<nav_msgs::Path>(dense_path_topic_, 1, true);
    anchor_path_pub_ = nh_.advertise<nav_msgs::Path>(anchor_path_topic_, 1, true);

    dense_path_pub_.publish(dense_path_msg_);
    anchor_path_pub_.publish(anchor_path_msg_);
    publishState(true);

    const double period = 1.0 / std::max(track_publish_rate_, 1.0);
    timer_ = nh_.createTimer(ros::Duration(period), &HybridRouteManager::controlLoop, this);

    ROS_INFO("HybridRouteManager ready. csv=%s raw_points=%zu dense_points=%zu state=%s",
             csv_path_.c_str(), raw_points_.size(), dense_points_.size(),
             stateToString(state_).c_str());
  }

 private:
  enum class State {
    kTrackRoute = 0,
  };

  void loadParams() {
    pose_topic_ = pnh_.param<std::string>("pose_topic", "/airsim_node/drone_1/debug/pose_gt");
    dp_cmd_topic_ = pnh_.param<std::string>("dp_cmd_topic", "/dp_planner/position_cmd");
    output_cmd_topic_ = pnh_.param<std::string>("output_cmd_topic", "/position_cmd");
    goal_topic_ = pnh_.param<std::string>("goal_topic", "/goal_pose");
    state_topic_ = pnh_.param<std::string>("state_topic", "/hybrid_route/state");
    dense_path_topic_ = pnh_.param<std::string>("dense_path_topic", "/hybrid_route/reference_path");
    anchor_path_topic_ = pnh_.param<std::string>("anchor_path_topic", "/hybrid_route/anchor_path");

    csv_path_ = pnh_.param<std::string>(
        "csv_path",
        std::string("src/sim_tools/path/rule_based_strict_route_anchors.csv"));

    route_spacing_ = pnh_.param("route_spacing", 0.50);
    dense_progress_search_window_ = pnh_.param("dense_progress_search_window", 160);
    raw_progress_search_window_ = pnh_.param("raw_progress_search_window", 36);
    progress_z_weight_ = pnh_.param("progress_z_weight", 0.20);

    track_look_ahead_distance_ = pnh_.param("track_look_ahead_distance", 4.5);
    track_cruise_speed_ = pnh_.param("track_cruise_speed", 4.2);
    track_slow_down_radius_ = pnh_.param("track_slow_down_radius", 10.0);
    track_min_speed_ = pnh_.param("track_min_speed", 1.0);
    track_min_dist_threshold_ = pnh_.param("track_min_dist_threshold", 0.15);
    track_max_climb_speed_ = pnh_.param("track_max_climb_speed", 3.0);
    track_z_speed_gain_ = pnh_.param("track_z_speed_gain", 2.0);
    track_publish_rate_ = pnh_.param("track_publish_rate", 50.0);
    goal_anchor_offset_ = pnh_.param("goal_anchor_offset", 1);
    goal_republish_dist_ = pnh_.param("goal_republish_dist", 0.5);
    goal_reached_radius_ = pnh_.param("goal_reached_radius", 6.0);
  }

  void loadRouteFromCsv() {
    std::ifstream fin(csv_path_.c_str());
    if (!fin.is_open()) {
      throw std::runtime_error("无法打开路径 CSV: " + csv_path_);
    }

    std::string header_line;
    if (!std::getline(fin, header_line)) {
      throw std::runtime_error("路径 CSV 为空: " + csv_path_);
    }

    const std::vector<std::string> header_cells = splitCsvLine(header_line);
    std::vector<std::string> header;
    header.reserve(header_cells.size());
    for (const std::string& cell : header_cells) {
      std::string lowered = trim(cell);
      std::transform(lowered.begin(), lowered.end(), lowered.begin(), ::tolower);
      header.push_back(lowered);
    }

    int x_idx = -1;
    int y_idx = -1;
    int z_idx = -1;
    int segment_idx = -1;
    int global_order_idx = -1;

    for (std::size_t i = 0; i < header.size(); ++i) {
      if (header[i] == "x") x_idx = static_cast<int>(i);
      if (header[i] == "y") y_idx = static_cast<int>(i);
      if (header[i] == "z") z_idx = static_cast<int>(i);
      if (header[i] == "segment") segment_idx = static_cast<int>(i);
      if (header[i] == "global_order") global_order_idx = static_cast<int>(i);
    }

    if (x_idx < 0 || y_idx < 0 || z_idx < 0) {
      throw std::runtime_error("CSV 缺少 x/y/z 列: " + csv_path_);
    }

    std::string line;
    while (std::getline(fin, line)) {
      if (trim(line).empty()) {
        continue;
      }

      const std::vector<std::string> cells = splitCsvLine(line);
      const int max_required_idx = std::max({x_idx, y_idx, z_idx});
      if (static_cast<int>(cells.size()) <= max_required_idx) {
        continue;
      }

      try {
        RoutePoint point;
        point.x = std::stod(trim(cells[x_idx]));
        point.y = std::stod(trim(cells[y_idx]));
        point.z = std::stod(trim(cells[z_idx]));
        if (segment_idx >= 0 && static_cast<int>(cells.size()) > segment_idx) {
          point.segment = trim(cells[segment_idx]);
        }
        if (global_order_idx >= 0 && static_cast<int>(cells.size()) > global_order_idx &&
            !trim(cells[global_order_idx]).empty()) {
          point.global_order = std::stod(trim(cells[global_order_idx]));
        }
        raw_points_.push_back(point);
      } catch (const std::exception&) {
        continue;
      }
    }

    if (raw_points_.size() < 2) {
      throw std::runtime_error("有效路径点太少: " + csv_path_);
    }

    std::sort(raw_points_.begin(), raw_points_.end(),
              [](const RoutePoint& lhs, const RoutePoint& rhs) {
                return lhs.global_order < rhs.global_order;
              });
  }

  void densifyRoute() {
    dense_points_.clear();
    dense_points_.push_back(raw_points_.front());
    raw_to_dense_idx_.clear();
    raw_to_dense_idx_.reserve(raw_points_.size());
    raw_to_dense_idx_.push_back(0);

    const double spacing = std::max(route_spacing_, 1e-3);
    for (std::size_t i = 0; i + 1 < raw_points_.size(); ++i) {
      const RoutePoint& start = raw_points_[i];
      const RoutePoint& end = raw_points_[i + 1];
      const double seg_len = spatialDistance(start, end);
      if (seg_len < 1e-6) {
        continue;
      }

      const int steps = std::max(static_cast<int>(std::ceil(seg_len / spacing)), 1);
      for (int step = 1; step <= steps; ++step) {
        const double alpha = static_cast<double>(step) / static_cast<double>(steps);
        RoutePoint point;
        point.x = start.x + (end.x - start.x) * alpha;
        point.y = start.y + (end.y - start.y) * alpha;
        point.z = start.z + (end.z - start.z) * alpha;
        point.segment = end.segment;
        point.global_order = start.global_order + (end.global_order - start.global_order) * alpha;
        dense_points_.push_back(point);
      }
      raw_to_dense_idx_.push_back(static_cast<int>(dense_points_.size()) - 1);
    }
  }

  void buildPathMessages() {
    dense_path_msg_.header.frame_id = "world";
    for (const RoutePoint& point : dense_points_) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = point.z;
      pose.pose.orientation.w = 1.0;
      dense_path_msg_.poses.push_back(pose);
    }

    anchor_path_msg_.header.frame_id = "world";
    for (const RoutePoint& point : raw_points_) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = point.z;
      pose.pose.orientation.w = 1.0;
      anchor_path_msg_.poses.push_back(pose);
    }
  }

  std::string stateToString(State state) const {
    switch (state) {
      case State::kTrackRoute:
      default:
        return "TRACK_ROUTE";
    }
  }

  void publishState(bool force_log) {
    std_msgs::String msg;
    msg.data = stateToString(state_);
    state_pub_.publish(msg);
    if (force_log) {
      ROS_INFO("HybridRouteManager state=%s", msg.data.c_str());
    }
  }

  void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    const ros::Time now_stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    if (has_pose_) {
      const double dt = std::max((now_stamp - last_pose_stamp_).toSec(), 1e-3);
      const double dx = msg->pose.position.x - current_pose_.pose.position.x;
      const double dy = msg->pose.position.y - current_pose_.pose.position.y;
      current_speed_xy_ = std::sqrt(dx * dx + dy * dy) / dt;
    }
    current_pose_ = *msg;
    has_pose_ = true;
    last_pose_stamp_ = now_stamp;

    const auto& q = msg->pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    current_yaw_ = normalizeAngle(std::atan2(siny_cosp, cosy_cosp));
  }

  void dpCmdCb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    latest_dp_cmd_ = *msg;
    has_dp_cmd_ = true;
  }

  int findNearestDenseIdx() {
    if (!has_pose_ || dense_points_.empty()) {
      return -1;
    }

    const double pose_x = current_pose_.pose.position.x;
    const double pose_y = current_pose_.pose.position.y;
    const double pose_z = current_pose_.pose.position.z;

    int start = 0;
    int end = static_cast<int>(dense_points_.size()) - 1;
    if (dense_progress_idx_ >= 0) {
      start = std::max(0, dense_progress_idx_);
      end = std::min(static_cast<int>(dense_points_.size()) - 1, dense_progress_idx_ + dense_progress_search_window_);
    }

    int best_idx = start;
    double best_score = std::numeric_limits<double>::infinity();
    for (int i = start; i <= end; ++i) {
      const RoutePoint& point = dense_points_[i];
      const double dx = point.x - pose_x;
      const double dy = point.y - pose_y;
      const double dz = point.z - pose_z;
      const double score = std::sqrt(dx * dx + dy * dy + std::pow(progress_z_weight_ * dz, 2.0));
      if (score < best_score) {
        best_score = score;
        best_idx = i;
      }
    }

    dense_progress_idx_ = best_idx;
    return best_idx;
  }

  int findNearestRawIdx() {
    if (!has_pose_ || raw_points_.empty()) {
      return -1;
    }

    const double pose_x = current_pose_.pose.position.x;
    const double pose_y = current_pose_.pose.position.y;
    const double pose_z = current_pose_.pose.position.z;

    int start = 0;
    int end = static_cast<int>(raw_points_.size()) - 1;
    if (raw_progress_idx_ >= 0) {
      start = std::max(0, raw_progress_idx_);
      end = std::min(static_cast<int>(raw_points_.size()) - 1, raw_progress_idx_ + raw_progress_search_window_);
    }

    int best_idx = start;
    double best_score = std::numeric_limits<double>::infinity();
    for (int i = start; i <= end; ++i) {
      const RoutePoint& point = raw_points_[i];
      const double dx = point.x - pose_x;
      const double dy = point.y - pose_y;
      const double dz = point.z - pose_z;
      const double score = std::sqrt(dx * dx + dy * dy + std::pow(progress_z_weight_ * dz, 2.0));
      if (score < best_score) {
        best_score = score;
        best_idx = i;
      }
    }

    raw_progress_idx_ = best_idx;
    return best_idx;
  }

  int advanceDenseIdxByDistance(int start_idx, double lookahead) const {
    int target_idx = start_idx;
    double accum = 0.0;
    for (int i = start_idx; i + 1 < static_cast<int>(dense_points_.size()); ++i) {
      const RoutePoint& p0 = dense_points_[i];
      const RoutePoint& p1 = dense_points_[i + 1];
      accum += horizontalDistance(p0, p1);
      target_idx = i + 1;
      if (accum >= lookahead) {
        break;
      }
    }

    return target_idx;
  }

  RoutePoint currentRoutePoint() const {
    RoutePoint curr;
    curr.x = current_pose_.pose.position.x;
    curr.y = current_pose_.pose.position.y;
    curr.z = current_pose_.pose.position.z;
    return curr;
  }


  quadrotor_msgs::PositionCommand buildTrackCommand(int dense_idx) {
    const int target_idx = advanceDenseIdxByDistance(dense_idx, track_look_ahead_distance_);
    RoutePoint target = dense_points_[target_idx];
    RoutePoint next = dense_points_[std::min(target_idx + 1, static_cast<int>(dense_points_.size()) - 1)];

    double dir_x = next.x - target.x;
    double dir_y = next.y - target.y;
    double dir_z = next.z - target.z;
    const double norm = std::sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    if (norm > 1e-6) {
      dir_x /= norm;
      dir_y /= norm;
      dir_z /= norm;
    } else {
      dir_x = 0.0;
      dir_y = 0.0;
      dir_z = 0.0;
    }

    const RoutePoint& route_end = dense_points_.back();
    RoutePoint current_point;
    current_point.x = current_pose_.pose.position.x;
    current_point.y = current_pose_.pose.position.y;
    current_point.z = current_pose_.pose.position.z;
    const double dist_to_end = horizontalDistance(current_point, route_end);

    double speed = track_cruise_speed_;
    if (track_slow_down_radius_ > 1e-3) {
      const double ratio = std::min(std::max(dist_to_end / track_slow_down_radius_, 0.0), 1.0);
      speed = std::max(track_min_speed_, track_cruise_speed_ * ratio);
    }

    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.position.x = target.x;
    cmd.position.y = target.y;
    cmd.position.z = target.z;
    cmd.velocity.x = speed * dir_x;
    cmd.velocity.y = speed * dir_y;
    cmd.velocity.z = std::max(-track_max_climb_speed_,
                              std::min(track_max_climb_speed_, speed * dir_z * track_z_speed_gain_));
    cmd.acceleration.x = 0.0;
    cmd.acceleration.y = 0.0;
    cmd.acceleration.z = 0.0;
    cmd.jerk.x = 0.0;
    cmd.jerk.y = 0.0;
    cmd.jerk.z = 0.0;
    cmd.kx = {0.0, 0.0, 0.0};
    cmd.kv = {0.0, 0.0, 0.0};

    const double dx = target.x - current_pose_.pose.position.x;
    const double dy = target.y - current_pose_.pose.position.y;
    const double dist_xy = std::sqrt(dx * dx + dy * dy);
    cmd.yaw = dist_xy > track_min_dist_threshold_ ? std::atan2(dy, dx) : current_yaw_;
    cmd.yaw_dot = 0.0;
    cmd.trajectory_id = 0;
    cmd.trajectory_flag = FLAG_TRACK;
    return cmd;
  }

  geometry_msgs::PoseStamped buildGoalPose(int raw_idx) {
    const int goal_offset = std::max(goal_anchor_offset_, 0);
    int goal_idx = std::min(raw_idx + goal_offset,
                            static_cast<int>(raw_points_.size()) - 1);
    goal_idx = std::max(0, std::min(goal_idx, static_cast<int>(raw_points_.size()) - 1));
    const RoutePoint& goal = raw_points_[goal_idx];
    const RoutePoint& heading_ref = raw_points_[std::min(goal_idx + 1, static_cast<int>(raw_points_.size()) - 1)];

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = goal.x;
    pose.pose.position.y = goal.y;
    pose.pose.position.z = goal.z;
    pose.pose.orientation = yawToQuaternion(std::atan2(heading_ref.y - goal.y, heading_ref.x - goal.x));
    return pose;
  }

  bool shouldPublishGoal(const geometry_msgs::PoseStamped& goal) const {
    if (!has_last_goal_) {
      return true;
    }

    const double dx = goal.pose.position.x - last_goal_.pose.position.x;
    const double dy = goal.pose.position.y - last_goal_.pose.position.y;
    const double dz = goal.pose.position.z - last_goal_.pose.position.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist >= goal_republish_dist_) {
      return true;
    }

    RoutePoint curr;
    curr.x = current_pose_.pose.position.x;
    curr.y = current_pose_.pose.position.y;
    curr.z = current_pose_.pose.position.z;
    RoutePoint goal_point;
    goal_point.x = goal.pose.position.x;
    goal_point.y = goal.pose.position.y;
    goal_point.z = goal.pose.position.z;
    return spatialDistance(curr, goal_point) <= goal_reached_radius_;
  }

  void controlLoop(const ros::TimerEvent&) {
    if (!has_pose_) {
      return;
    }

    const int dense_idx = findNearestDenseIdx();
    const int raw_idx = findNearestRawIdx();
    if (dense_idx < 0 || raw_idx < 0) {
      return;
    }

    const geometry_msgs::PoseStamped goal = buildGoalPose(raw_idx);
    const bool goal_published = shouldPublishGoal(goal);
    if (goal_published) {
      goal_pub_.publish(goal);
      last_goal_ = goal;
      has_last_goal_ = true;
    }

    quadrotor_msgs::PositionCommand cmd = buildTrackCommand(dense_idx);
    last_track_cmd_ = cmd;
    has_last_track_cmd_ = true;
    cmd_pub_.publish(cmd);
    ROS_INFO_THROTTLE(0.5,
                      "HybridRoute TRACK dense=%d raw=%d goal_pub=%s target=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f)",
                      dense_idx, raw_idx, goal_published ? "YES" : "NO",
                      cmd.position.x, cmd.position.y, cmd.position.z,
                      cmd.velocity.x, cmd.velocity.y, cmd.velocity.z);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber pose_sub_;
  ros::Subscriber dp_cmd_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher state_pub_;
  ros::Publisher dense_path_pub_;
  ros::Publisher anchor_path_pub_;
  ros::Timer timer_;

  std::string pose_topic_;
  std::string dp_cmd_topic_;
  std::string output_cmd_topic_;
  std::string goal_topic_;
  std::string state_topic_;
  std::string dense_path_topic_;
  std::string anchor_path_topic_;
  std::string csv_path_;

  double route_spacing_ = 0.5;
  int dense_progress_search_window_ = 160;
  int raw_progress_search_window_ = 36;
  double progress_z_weight_ = 0.2;
  double track_look_ahead_distance_ = 4.5;
  double track_cruise_speed_ = 4.2;
  double track_slow_down_radius_ = 10.0;
  double track_min_speed_ = 1.0;
  double track_min_dist_threshold_ = 0.15;
  double track_max_climb_speed_ = 3.0;
  double track_z_speed_gain_ = 2.0;
  double track_publish_rate_ = 50.0;
  int goal_anchor_offset_ = 1;
  double goal_republish_dist_ = 0.5;
  double goal_reached_radius_ = 6.0;

  geometry_msgs::PoseStamped current_pose_;
  bool has_pose_ = false;
  double current_yaw_ = 0.0;
  double current_speed_xy_ = 0.0;
  ros::Time last_pose_stamp_;
  quadrotor_msgs::PositionCommand latest_dp_cmd_;
  bool has_dp_cmd_ = false;
  quadrotor_msgs::PositionCommand last_track_cmd_;
  bool has_last_track_cmd_ = false;
  geometry_msgs::PoseStamped last_goal_;
  bool has_last_goal_ = false;
  std::vector<RoutePoint> raw_points_;
  std::vector<RoutePoint> dense_points_;
  std::vector<int> raw_to_dense_idx_;
  nav_msgs::Path dense_path_msg_;
  nav_msgs::Path anchor_path_msg_;

  int dense_progress_idx_ = -1;
  int raw_progress_idx_ = -1;
  State state_ = State::kTrackRoute;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "hybrid_route_manager");

  try {
    HybridRouteManager manager;
    ros::spin();
  } catch (const std::exception& exc) {
    ROS_FATAL("HybridRouteManager init failed: %s", exc.what());
    return 1;
  }

  return 0;
}
