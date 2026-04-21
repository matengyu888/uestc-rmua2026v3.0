#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <string>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class StereoDepthNode {
 public:
  StereoDepthNode() : nh_(), pnh_("~") {
    left_topic_ = pnh_.param<std::string>("left_topic", "/airsim_node/drone_1/front_left/Scene");
    right_topic_ = pnh_.param<std::string>("right_topic", "/airsim_node/drone_1/front_right/Scene");
    left_info_topic_ = pnh_.param<std::string>("left_info_topic", "/airsim_node/drone_1/front_left/Scene/camera_info");
    right_info_topic_ = pnh_.param<std::string>("right_info_topic", "/airsim_node/drone_1/front_right/Scene/camera_info");
    lidar_topic_ = pnh_.param<std::string>("lidar_topic", "/airsim_node/drone_1/lidar");
    depth_topic_ = pnh_.param<std::string>("depth_topic", "/airsim_node/drone_1/front_stereo/DepthPerspective");

    baseline_m_ = pnh_.param("baseline_m", 0.30);
    camera_x_ = pnh_.param("camera_x", 0.175);
    camera_y_ = pnh_.param("camera_y", -0.15);
    camera_z_ = pnh_.param("camera_z", 0.0);
    lidar_x_ = pnh_.param("lidar_x", 0.0);
    lidar_y_ = pnh_.param("lidar_y", 0.0);
    lidar_z_ = pnh_.param("lidar_z", -0.05);
    fallback_width_ = pnh_.param("fallback_width", 960);
    fallback_height_ = pnh_.param("fallback_height", 720);
    fallback_fov_deg_ = pnh_.param("fallback_fov_deg", 60.0);
    max_sync_dt_ = pnh_.param("max_sync_dt", 0.08);
    min_depth_m_ = pnh_.param("min_depth_m", 0.3);
    max_depth_m_ = pnh_.param("max_depth_m", 24.0);
    publish_rate_ = pnh_.param("publish_rate", 15.0);
    block_size_ = makeOdd(pnh_.param("block_size", 5));
    num_disparities_ = alignTo16(std::max(16, pnh_.param("num_disparities", 128)));
    min_disparity_ = pnh_.param("min_disparity", 0);
    uniqueness_ratio_ = pnh_.param("uniqueness_ratio", 8);
    speckle_window_size_ = pnh_.param("speckle_window_size", 50);
    speckle_range_ = pnh_.param("speckle_range", 2);
    disp12_max_diff_ = pnh_.param("disp12_max_diff", 1);
    pre_filter_cap_ = pnh_.param("pre_filter_cap", 31);
    texture_threshold_ = pnh_.param("texture_threshold", 10);
    lidar_patch_radius_ = pnh_.param("lidar_patch_radius", 2);
    lidar_sync_dt_ = pnh_.param("lidar_sync_dt", 0.15);

    left_sub_ = nh_.subscribe(left_topic_, 1, &StereoDepthNode::leftCb, this);
    right_sub_ = nh_.subscribe(right_topic_, 1, &StereoDepthNode::rightCb, this);
    left_info_sub_ = nh_.subscribe(left_info_topic_, 1, &StereoDepthNode::leftInfoCb, this);
    right_info_sub_ = nh_.subscribe(right_info_topic_, 1, &StereoDepthNode::rightInfoCb, this);
    lidar_sub_ = nh_.subscribe(lidar_topic_, 1, &StereoDepthNode::lidarCb, this);
    depth_pub_ = nh_.advertise<sensor_msgs::Image>(depth_topic_, 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_)),
                             &StereoDepthNode::processTimer, this);

    buildMatcher();

    ROS_INFO("StereoDepthNode ready. left=%s right=%s lidar=%s depth=%s baseline=%.3f",
             left_topic_.c_str(), right_topic_.c_str(), lidar_topic_.c_str(), depth_topic_.c_str(), baseline_m_);
  }

 private:
  static int makeOdd(int value) {
    value = std::max(3, value);
    return (value % 2 == 0) ? value + 1 : value;
  }

  static int alignTo16(int value) {
    return ((value + 15) / 16) * 16;
  }

  void buildMatcher() {
    matcher_ = cv::StereoSGBM::create(min_disparity_, num_disparities_, block_size_);
    matcher_->setP1(8 * block_size_ * block_size_);
    matcher_->setP2(32 * block_size_ * block_size_);
    matcher_->setPreFilterCap(pre_filter_cap_);
    matcher_->setUniquenessRatio(uniqueness_ratio_);
    matcher_->setSpeckleWindowSize(speckle_window_size_);
    matcher_->setSpeckleRange(speckle_range_);
    matcher_->setDisp12MaxDiff(disp12_max_diff_);
    matcher_->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
  }

  void leftCb(const sensor_msgs::ImageConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    left_queue_.push_back(msg);
    trimQueue(&left_queue_);
  }

  void rightCb(const sensor_msgs::ImageConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    right_queue_.push_back(msg);
    trimQueue(&right_queue_);
  }

  void leftInfoCb(const sensor_msgs::CameraInfoConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    left_info_msg_ = msg;
  }

  void rightInfoCb(const sensor_msgs::CameraInfoConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    right_info_msg_ = msg;
  }

  void lidarCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    lidar_msg_ = msg;
  }

  void trimQueue(std::deque<sensor_msgs::ImageConstPtr>* queue) const {
    while (queue->size() > max_queue_size_) {
      queue->pop_front();
    }
  }

  bool chooseStereoPairLocked(sensor_msgs::ImageConstPtr* left_msg,
                              sensor_msgs::ImageConstPtr* right_msg) {
    if (left_queue_.empty() || right_queue_.empty()) {
      return false;
    }

    double best_dt = std::numeric_limits<double>::infinity();
    std::size_t best_left_idx = 0;
    std::size_t best_right_idx = 0;

    for (std::size_t li = 0; li < left_queue_.size(); ++li) {
      const ros::Time left_stamp = left_queue_[li]->header.stamp.isZero() ? ros::Time::now() : left_queue_[li]->header.stamp;
      for (std::size_t ri = 0; ri < right_queue_.size(); ++ri) {
        const ros::Time right_stamp = right_queue_[ri]->header.stamp.isZero() ? ros::Time::now() : right_queue_[ri]->header.stamp;
        const double dt = std::fabs((left_stamp - right_stamp).toSec());
        if (dt < best_dt) {
          best_dt = dt;
          best_left_idx = li;
          best_right_idx = ri;
        }
      }
    }

    if (best_dt > max_sync_dt_) {
      ROS_WARN_THROTTLE(1.0, "StereoDepth image sync too large: %.4f s", best_dt);
      return false;
    }

    *left_msg = left_queue_[best_left_idx];
    *right_msg = right_queue_[best_right_idx];
    left_queue_.erase(left_queue_.begin(), left_queue_.begin() + static_cast<long>(best_left_idx + 1));
    right_queue_.erase(right_queue_.begin(), right_queue_.begin() + static_cast<long>(best_right_idx + 1));
    return true;
  }

  double resolveFxLocked() const {
    if (left_info_msg_ && left_info_msg_->K[0] > 1e-6) {
      return left_info_msg_->K[0];
    }
    const double fov_rad = fallback_fov_deg_ * M_PI / 180.0;
    return static_cast<double>(fallback_width_) / (2.0 * std::tan(0.5 * fov_rad));
  }

  double resolveFyLocked() const {
    if (left_info_msg_ && left_info_msg_->K[4] > 1e-6) {
      return left_info_msg_->K[4];
    }
    return resolveFxLocked();
  }

  double resolveCxLocked() const {
    if (left_info_msg_ && std::isfinite(left_info_msg_->K[2])) {
      return left_info_msg_->K[2];
    }
    return static_cast<double>(fallback_width_) * 0.5;
  }

  double resolveCyLocked() const {
    if (left_info_msg_ && std::isfinite(left_info_msg_->K[5])) {
      return left_info_msg_->K[5];
    }
    return static_cast<double>(fallback_height_) * 0.5;
  }

  bool convertToGray(const sensor_msgs::ImageConstPtr& msg, cv::Mat* gray) const {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
      if (cv_ptr->image.empty()) {
        return false;
      }

      if (cv_ptr->image.channels() == 1) {
        if (cv_ptr->image.type() != CV_8UC1) {
          cv::Mat tmp;
          cv_ptr->image.convertTo(tmp, CV_8UC1);
          *gray = tmp;
        } else {
          *gray = cv_ptr->image;
        }
        return true;
      }

      if (cv_ptr->image.channels() == 3) {
        cv::cvtColor(cv_ptr->image, *gray, cv::COLOR_BGR2GRAY);
        return true;
      }

      if (cv_ptr->image.channels() == 4) {
        cv::cvtColor(cv_ptr->image, *gray, cv::COLOR_BGRA2GRAY);
        return true;
      }
    } catch (const cv_bridge::Exception& exc) {
      ROS_WARN_THROTTLE(1.0, "StereoDepth cv_bridge failed: %s", exc.what());
    }
    return false;
  }

  void processTimer(const ros::TimerEvent&) {
    sensor_msgs::ImageConstPtr left_msg;
    sensor_msgs::ImageConstPtr right_msg;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    sensor_msgs::PointCloud2ConstPtr lidar_msg;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!chooseStereoPairLocked(&left_msg, &right_msg)) {
        return;
      }
      fx = resolveFxLocked();
      fy = resolveFyLocked();
      cx = resolveCxLocked();
      cy = resolveCyLocked();
      lidar_msg = lidar_msg_;
    }

    cv::Mat left_gray;
    cv::Mat right_gray;
    if (!convertToGray(left_msg, &left_gray) || !convertToGray(right_msg, &right_gray)) {
      ROS_WARN_THROTTLE(1.0, "StereoDepth failed to convert stereo images to gray.");
      return;
    }

    if (left_gray.size() != right_gray.size()) {
      ROS_WARN_THROTTLE(1.0, "StereoDepth left/right image size mismatch.");
      return;
    }

    cv::Mat left_resized;
    cv::Mat right_resized;
    if (left_gray.cols != fallback_width_ || left_gray.rows != fallback_height_) {
      cv::resize(left_gray, left_resized, cv::Size(fallback_width_, fallback_height_));
      cv::resize(right_gray, right_resized, cv::Size(fallback_width_, fallback_height_));
    } else {
      left_resized = left_gray;
      right_resized = right_gray;
    }

    cv::Mat disparity16;
    matcher_->compute(left_resized, right_resized, disparity16);

    cv::Mat disparity32;
    disparity16.convertTo(disparity32, CV_32FC1, 1.0 / 16.0);

    cv::Mat depth(left_resized.rows, left_resized.cols, CV_32FC1,
                  cv::Scalar(std::numeric_limits<float>::quiet_NaN()));

    for (int r = 0; r < disparity32.rows; ++r) {
      const float* disp_row = disparity32.ptr<float>(r);
      float* depth_row = depth.ptr<float>(r);
      for (int c = 0; c < disparity32.cols; ++c) {
        const float disp = disp_row[c];
        if (disp <= 0.1f || !std::isfinite(disp)) {
          continue;
        }
        const float z = static_cast<float>(fx * baseline_m_ / disp);
        if (z < min_depth_m_ || z > max_depth_m_ || !std::isfinite(z)) {
          continue;
        }
        depth_row[c] = z;
      }
    }

    if (lidar_msg) {
      const ros::Time left_stamp = left_msg->header.stamp.isZero() ? ros::Time::now() : left_msg->header.stamp;
      const ros::Time lidar_stamp = lidar_msg->header.stamp.isZero() ? left_stamp : lidar_msg->header.stamp;
      const double dt = std::fabs((left_stamp - lidar_stamp).toSec());
      if (dt <= lidar_sync_dt_) {
        const double scale_x = static_cast<double>(fallback_width_) / static_cast<double>(left_gray.cols);
        const double scale_y = static_cast<double>(fallback_height_) / static_cast<double>(left_gray.rows);
        fuseLidarIntoDepth(*lidar_msg, fx * scale_x, fy * scale_y, cx * scale_x, cy * scale_y, &depth);
      } else {
        ROS_WARN_THROTTLE(1.0, "StereoDepth lidar sync too large: %.4f s", dt);
      }
    }

    cv::medianBlur(depth, depth, 5);

    cv_bridge::CvImage out;
    out.header = left_msg->header;
    out.encoding = "32FC1";
    out.image = depth;
    depth_pub_.publish(out.toImageMsg());
  }

  void fuseLidarIntoDepth(const sensor_msgs::PointCloud2& cloud,
                          double fx,
                          double fy,
                          double cx,
                          double cy,
                          cv::Mat* depth) const {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    const double rel_x = lidar_x_ - camera_x_;
    const double rel_y = lidar_y_ - camera_y_;
    const double rel_z = lidar_z_ - camera_z_;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }

      const double x_cam = static_cast<double>(*iter_x) + rel_x;
      const double y_cam = static_cast<double>(*iter_y) + rel_y;
      const double z_cam = static_cast<double>(*iter_z) + rel_z;

      if (x_cam <= min_depth_m_ || x_cam >= max_depth_m_) {
        continue;
      }

      const int u = static_cast<int>(std::lround(fx * (y_cam / x_cam) + cx));
      const int v = static_cast<int>(std::lround(fy * (z_cam / x_cam) + cy));
      if (u < 0 || u >= depth->cols || v < 0 || v >= depth->rows) {
        continue;
      }

      for (int dv = -lidar_patch_radius_; dv <= lidar_patch_radius_; ++dv) {
        const int vv = v + dv;
        if (vv < 0 || vv >= depth->rows) {
          continue;
        }
        float* row = depth->ptr<float>(vv);
        for (int du = -lidar_patch_radius_; du <= lidar_patch_radius_; ++du) {
          const int uu = u + du;
          if (uu < 0 || uu >= depth->cols) {
            continue;
          }
          if (!std::isfinite(row[uu]) || x_cam < row[uu]) {
            row[uu] = static_cast<float>(x_cam);
          }
        }
      }
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber left_sub_;
  ros::Subscriber right_sub_;
  ros::Subscriber left_info_sub_;
  ros::Subscriber right_info_sub_;
  ros::Subscriber lidar_sub_;
  ros::Publisher depth_pub_;
  ros::Timer timer_;

  std::string left_topic_;
  std::string right_topic_;
  std::string left_info_topic_;
  std::string right_info_topic_;
  std::string lidar_topic_;
  std::string depth_topic_;

  double baseline_m_ = 0.30;
  double camera_x_ = 0.175;
  double camera_y_ = -0.15;
  double camera_z_ = 0.0;
  double lidar_x_ = 0.0;
  double lidar_y_ = 0.0;
  double lidar_z_ = -0.05;
  int fallback_width_ = 960;
  int fallback_height_ = 720;
  double fallback_fov_deg_ = 60.0;
  double max_sync_dt_ = 0.08;
  double min_depth_m_ = 0.3;
  double max_depth_m_ = 24.0;
  double publish_rate_ = 15.0;
  int block_size_ = 5;
  int num_disparities_ = 128;
  int min_disparity_ = 0;
  int uniqueness_ratio_ = 8;
  int speckle_window_size_ = 50;
  int speckle_range_ = 2;
  int disp12_max_diff_ = 1;
  int pre_filter_cap_ = 31;
  int texture_threshold_ = 10;
  int lidar_patch_radius_ = 2;
  double lidar_sync_dt_ = 0.15;
  std::size_t max_queue_size_ = 6;

  cv::Ptr<cv::StereoSGBM> matcher_;

  mutable std::mutex data_mutex_;
  std::deque<sensor_msgs::ImageConstPtr> left_queue_;
  std::deque<sensor_msgs::ImageConstPtr> right_queue_;
  sensor_msgs::CameraInfoConstPtr left_info_msg_;
  sensor_msgs::CameraInfoConstPtr right_info_msg_;
  sensor_msgs::PointCloud2ConstPtr lidar_msg_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "stereo_depth_node");
  StereoDepthNode node;
  ros::spin();
  return 0;
}
