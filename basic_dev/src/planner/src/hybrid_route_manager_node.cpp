#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
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
  HybridRouteManager() : nh_(), pnh_("~") {
    loadParams();
    loadRouteFromCsv();
    densifyRoute();
    buildSegments();
    buildPathMessages();

    pose_sub_ = nh_.subscribe(pose_topic_, 1, &HybridRouteManager::poseCb, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1, true);
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
  struct RouteSegment {
    int raw_start_idx = 0;
    int raw_end_idx = 0;
    int dense_start_idx = 0;
    int dense_end_idx = 0;
  };

  enum class State {
    kDpPlanner = 0,
  };

  void loadParams() {
    pose_topic_ = pnh_.param<std::string>("pose_topic", "/airsim_node/drone_1/debug/pose_gt");
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

    track_publish_rate_ = pnh_.param("track_publish_rate", 50.0);
    goal_anchor_offset_ = pnh_.param("goal_anchor_offset", 1);
    goal_republish_dist_ = pnh_.param("goal_republish_dist", 0.5);
    goal_reached_radius_ = pnh_.param("goal_reached_radius", 6.0);
    dp_goal_corridor_lookahead_ = pnh_.param("dp_goal_corridor_lookahead", 8.0);
    gate_slowdown_distance_ = pnh_.param("gate_slowdown_distance", 5.0);
    segment_entry_hold_distance_ = pnh_.param("segment_entry_hold_distance", 4.0);
    segment_entry_activation_radius_ = pnh_.param("segment_entry_activation_radius", 12.0);
    segment_entry_goal_lookahead_ = pnh_.param("segment_entry_goal_lookahead", 2.0);
    duplicate_gate_threshold_ = pnh_.param("duplicate_gate_threshold", 0.30);
    segment_switch_radius_ = pnh_.param("segment_switch_radius", 2.0);
    gate_pass_margin_ = pnh_.param("gate_pass_margin", 0.3);
    gate_pass_activation_radius_ = pnh_.param("gate_pass_activation_radius", 12.0);
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

  void buildSegments() {
    segments_.clear();
    if (raw_points_.empty()) {
      return;
    }

    int seg_start = 0;
    for (int i = 0; i + 1 < static_cast<int>(raw_points_.size()); ++i) {
      if (spatialDistance(raw_points_[i], raw_points_[i + 1]) > duplicate_gate_threshold_) {
        continue;
      }

      RouteSegment seg;
      seg.raw_start_idx = seg_start;
      seg.raw_end_idx = i;
      seg.dense_start_idx = raw_toDenseClamped(seg.raw_start_idx);
      seg.dense_end_idx = raw_toDenseClamped(seg.raw_end_idx);
      segments_.push_back(seg);
      seg_start = i + 1;
    }

    RouteSegment final_seg;
    final_seg.raw_start_idx = seg_start;
    final_seg.raw_end_idx = static_cast<int>(raw_points_.size()) - 1;
    final_seg.dense_start_idx = raw_toDenseClamped(final_seg.raw_start_idx);
    final_seg.dense_end_idx = raw_toDenseClamped(final_seg.raw_end_idx);
    segments_.push_back(final_seg);

    active_segment_idx_ = 0;
    entry_gate_released_ = false;
  }

  int raw_toDenseClamped(int raw_idx) const {
    if (raw_to_dense_idx_.empty()) {
      return 0;
    }
    raw_idx = std::max(0, std::min(raw_idx, static_cast<int>(raw_to_dense_idx_.size()) - 1));
    return raw_to_dense_idx_[raw_idx];
  }

  const RouteSegment& activeSegment() const {
    return segments_[std::max(0, std::min(active_segment_idx_, static_cast<int>(segments_.size()) - 1))];
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
      case State::kDpPlanner:
        return "DP_PLANNER";
      default:
        return "DP_PLANNER";
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
    current_pose_ = *msg;
    has_pose_ = true;
    if (!boot_goal_published_) {
      publishGoalIfPossible(true, "boot");
    }
  }

  bool publishGoalIfPossible(bool force_publish, const char* reason) {
    const int dense_idx = findNearestDenseIdx();
    const int raw_idx = findNearestRawIdx();
    if (dense_idx < 0 || raw_idx < 0) {
      ROS_WARN_THROTTLE(1.0, "HybridRouteManager cannot build goal yet. reason=%s has_pose=%s dense_points=%zu raw_points=%zu",
                        reason, has_pose_ ? "true" : "false", dense_points_.size(), raw_points_.size());
      return false;
    }

    maybeAdvanceSegment();

    const int segment_dense_idx = findNearestDenseIdx();
    const int segment_raw_idx = findNearestRawIdx();
    if (segment_dense_idx < 0 || segment_raw_idx < 0) {
      ROS_WARN_THROTTLE(1.0, "HybridRouteManager segment-local goal lookup failed. reason=%s", reason);
      return false;
    }

    const geometry_msgs::PoseStamped goal = buildGoalPose(segment_raw_idx);
    const bool goal_published = force_publish || shouldPublishGoal(goal);
    if (goal_published) {
      goal_pub_.publish(goal);
      last_goal_ = goal;
      has_last_goal_ = true;
      last_goal_publish_time_ = ros::Time::now();
      boot_goal_published_ = true;
    }

    const RouteSegment& segment = activeSegment();
    ROS_INFO_THROTTLE(0.5,
                      "HybridRoute GOAL seg=%d/%zu raw=%d dense=%d range_raw=[%d,%d] reason=%s goal_pub=%s goal=(%.2f,%.2f,%.2f)",
                      active_segment_idx_ + 1, segments_.size(),
                      segment_raw_idx, segment_dense_idx,
                      segment.raw_start_idx, segment.raw_end_idx,
                      reason,
                      goal_published ? "YES" : "NO",
                      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    return goal_published;
  }

  int findNearestDenseIdx() {
    if (!has_pose_ || dense_points_.empty() || segments_.empty()) {
      return -1;
    }

    const double pose_x = current_pose_.pose.position.x;
    const double pose_y = current_pose_.pose.position.y;
    const double pose_z = current_pose_.pose.position.z;
    const RouteSegment& segment = activeSegment();

    int start = segment.dense_start_idx;
    int end = segment.dense_end_idx;
    if (dense_progress_idx_ >= 0) {
      start = std::max(segment.dense_start_idx, dense_progress_idx_);
      end = std::min(segment.dense_end_idx, dense_progress_idx_ + dense_progress_search_window_);
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
    if (!has_pose_ || raw_points_.empty() || segments_.empty()) {
      return -1;
    }

    const double pose_x = current_pose_.pose.position.x;
    const double pose_y = current_pose_.pose.position.y;
    const double pose_z = current_pose_.pose.position.z;
    const RouteSegment& segment = activeSegment();

    int start = segment.raw_start_idx;
    int end = segment.raw_end_idx;
    if (raw_progress_idx_ >= 0) {
      start = std::max(segment.raw_start_idx, raw_progress_idx_);
      end = std::min(segment.raw_end_idx, raw_progress_idx_ + raw_progress_search_window_);
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
    if (segments_.empty()) {
      return start_idx;
    }
    const RouteSegment& segment = activeSegment();
    start_idx = std::max(segment.dense_start_idx, std::min(start_idx, segment.dense_end_idx));
    int target_idx = start_idx;
    double accum = 0.0;
    for (int i = start_idx; i + 1 <= segment.dense_end_idx; ++i) {
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

  geometry_msgs::PoseStamped buildGoalPose(int raw_idx) {
    RoutePoint goal;
    RoutePoint heading_ref;
    const RouteSegment& segment = activeSegment();
    bool use_gate_approach_heading = false;
    bool hold_segment_entry = false;

    RoutePoint curr;
    curr.x = current_pose_.pose.position.x;
    curr.y = current_pose_.pose.position.y;
    curr.z = current_pose_.pose.position.z;

    if (active_segment_idx_ > 0 && segment.raw_end_idx > segment.raw_start_idx) {
      const RoutePoint& entry_gate = raw_points_[segment.raw_start_idx];
      const RoutePoint& next_ref = raw_points_[segment.raw_start_idx + 1];
      const double dir_x = next_ref.x - entry_gate.x;
      const double dir_y = next_ref.y - entry_gate.y;
      const double dir_z = next_ref.z - entry_gate.z;
      const double dir_norm = std::sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
      if (!entry_gate_released_ &&
          dir_norm > 1e-6 &&
          spatialDistance(curr, entry_gate) <= segment_entry_activation_radius_) {
        const double unit_x = dir_x / dir_norm;
        const double unit_y = dir_y / dir_norm;
        const double unit_z = dir_z / dir_norm;
        const double along_track =
            (curr.x - entry_gate.x) * unit_x +
            (curr.y - entry_gate.y) * unit_y +
            (curr.z - entry_gate.z) * unit_z;
        const bool reached_entry_gate =
            spatialDistance(curr, entry_gate) <= std::max(goal_republish_dist_, 0.3);
        if (reached_entry_gate) {
          entry_gate_released_ = true;
        }
        hold_segment_entry = !reached_entry_gate && along_track < segment_entry_hold_distance_;
        if (hold_segment_entry) {
          goal = entry_gate;
          heading_ref = next_ref;
        }
      }
    }

    if (hold_segment_entry) {
      use_gate_approach_heading = false;
    } else if (!dense_points_.empty() && dense_progress_idx_ >= 0) {
      const RoutePoint& gate = raw_points_[segment.raw_end_idx];
      const bool approaching_gate = spatialDistance(curr, gate) <= gate_slowdown_distance_;
      if (approaching_gate) {
        goal = gate;
        use_gate_approach_heading = true;
      } else {
        const int dense_goal_idx =
            advanceDenseIdxByDistance(std::max(0, dense_progress_idx_), dp_goal_corridor_lookahead_);
        const int heading_idx = std::min(dense_goal_idx + 1, segment.dense_end_idx);
        goal = dense_points_[dense_goal_idx];
        heading_ref = dense_points_[heading_idx];
      }
    } else {
      int goal_idx = std::max(segment.raw_start_idx, raw_idx);
      const int goal_offset = std::max(goal_anchor_offset_, 0);
      goal_idx = std::min(goal_idx + goal_offset, segment.raw_end_idx);
      goal = raw_points_[goal_idx];
      heading_ref = raw_points_[std::min(goal_idx + 1, segment.raw_end_idx)];
    }

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = goal.x;
    pose.pose.position.y = goal.y;
    pose.pose.position.z = goal.z;
    double goal_yaw = 0.0;
    if (use_gate_approach_heading) {
      const int prev_idx = std::max(segment.raw_start_idx, segment.raw_end_idx - 1);
      const RoutePoint& prev = raw_points_[prev_idx];
      goal_yaw = std::atan2(goal.y - prev.y, goal.x - prev.x);
    } else {
      goal_yaw = std::atan2(heading_ref.y - goal.y, heading_ref.x - goal.x);
    }
    pose.pose.orientation = yawToQuaternion(goal_yaw);
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

  bool hasPassedGate(const RouteSegment& segment, const RoutePoint& curr) const {
    if (segment.raw_end_idx <= segment.raw_start_idx) {
      return false;
    }

    const RoutePoint& gate = raw_points_[segment.raw_end_idx];
    if (spatialDistance(curr, gate) > gate_pass_activation_radius_) {
      return false;
    }

    const RoutePoint& prev = raw_points_[segment.raw_end_idx - 1];
    const double dir_x = gate.x - prev.x;
    const double dir_y = gate.y - prev.y;
    const double dir_z = gate.z - prev.z;
    const double dir_norm = std::sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    if (dir_norm < 1e-6) {
      return false;
    }

    const double unit_x = dir_x / dir_norm;
    const double unit_y = dir_y / dir_norm;
    const double unit_z = dir_z / dir_norm;

    const double gate_to_curr_x = curr.x - gate.x;
    const double gate_to_curr_y = curr.y - gate.y;
    const double gate_to_curr_z = curr.z - gate.z;
    const double along_track =
        gate_to_curr_x * unit_x + gate_to_curr_y * unit_y + gate_to_curr_z * unit_z;

    return along_track >= gate_pass_margin_;
  }

  void maybeAdvanceSegment() {
    if (segments_.empty() || active_segment_idx_ + 1 >= static_cast<int>(segments_.size()) || !has_pose_) {
      return;
    }

    const RouteSegment& segment = activeSegment();
    const RoutePoint& gate = raw_points_[segment.raw_end_idx];

    RoutePoint curr;
    curr.x = current_pose_.pose.position.x;
    curr.y = current_pose_.pose.position.y;
    curr.z = current_pose_.pose.position.z;

    const bool reached_gate = spatialDistance(curr, gate) <= segment_switch_radius_;
    const bool passed_gate = hasPassedGate(segment, curr);
    if (!(reached_gate || passed_gate)) {
      return;
    }

    ++active_segment_idx_;
    const RouteSegment& next_segment = activeSegment();
    raw_progress_idx_ = std::max(raw_progress_idx_, next_segment.raw_start_idx);
    dense_progress_idx_ = std::max(dense_progress_idx_, next_segment.dense_start_idx);
    has_last_goal_ = false;
    entry_gate_released_ = false;

    ROS_INFO("HybridRouteManager advanced to segment %d/%zu gate_raw=%d next_raw=[%d,%d] reached_gate=%s passed_gate=%s",
             active_segment_idx_ + 1, segments_.size(), segment.raw_end_idx,
             next_segment.raw_start_idx, next_segment.raw_end_idx,
             reached_gate ? "true" : "false",
             passed_gate ? "true" : "false");
  }

  void controlLoop(const ros::TimerEvent&) {
    if (!has_pose_) {
      return;
    }
    const bool stale_goal =
        !last_goal_publish_time_.isValid() ||
        (ros::Time::now() - last_goal_publish_time_).toSec() > forced_goal_refresh_sec_;
    publishGoalIfPossible(stale_goal, stale_goal ? "timer_force" : "timer");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber pose_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher state_pub_;
  ros::Publisher dense_path_pub_;
  ros::Publisher anchor_path_pub_;
  ros::Timer timer_;

  std::string pose_topic_;
  std::string goal_topic_;
  std::string state_topic_;
  std::string dense_path_topic_;
  std::string anchor_path_topic_;
  std::string csv_path_;

  double route_spacing_ = 0.5;
  int dense_progress_search_window_ = 160;
  int raw_progress_search_window_ = 36;
  double progress_z_weight_ = 0.2;
  double track_publish_rate_ = 50.0;
  int goal_anchor_offset_ = 1;
  double goal_republish_dist_ = 0.5;
  double goal_reached_radius_ = 6.0;
  double dp_goal_corridor_lookahead_ = 8.0;
  double gate_slowdown_distance_ = 5.0;
  double segment_entry_hold_distance_ = 4.0;
  double segment_entry_activation_radius_ = 12.0;
  double segment_entry_goal_lookahead_ = 2.0;
  double duplicate_gate_threshold_ = 0.3;
  double segment_switch_radius_ = 2.0;
  double gate_pass_margin_ = 0.3;
  double gate_pass_activation_radius_ = 12.0;

  geometry_msgs::PoseStamped current_pose_;
  bool has_pose_ = false;
  geometry_msgs::PoseStamped last_goal_;
  bool has_last_goal_ = false;
  bool boot_goal_published_ = false;
  ros::Time last_goal_publish_time_;
  std::vector<RoutePoint> raw_points_;
  std::vector<RoutePoint> dense_points_;
  std::vector<int> raw_to_dense_idx_;
  std::vector<RouteSegment> segments_;
  nav_msgs::Path dense_path_msg_;
  nav_msgs::Path anchor_path_msg_;

  int dense_progress_idx_ = -1;
  int raw_progress_idx_ = -1;
  int active_segment_idx_ = 0;
  bool entry_gate_released_ = false;
  double forced_goal_refresh_sec_ = 0.5;
  State state_ = State::kDpPlanner;
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
