#include <cstdio>
#include <cmath>
#include <iostream>
#include "metaflylib/elements.hpp"
#include "metaflylib/poseprocessing.hpp"

namespace metaflylib
{
  PoseProcessor::PoseProcessor(size_t history_size, double max_identical_seconds, Box bounding_box, Ellipse control_ellipse) 
  : history_size_(history_size), max_identical_seconds_(max_identical_seconds), bounding_box_(bounding_box), control_ellipse_(control_ellipse) {
    if (history_size_ <= 0) {
      throw std::invalid_argument("History size must be greater than 0");
    }
    if (max_identical_seconds_ <= 0) {
      throw std::invalid_argument("Max identical seconds must be greater than 0");
    }
  }

  bool operator==(const Pose & lhs, const Pose & rhs) {
    return (almost_equal(lhs.x, rhs.x) &&
            almost_equal(lhs.y, rhs.y) &&
            almost_equal(lhs.z, rhs.z) &&
            lhs.roll == rhs.roll &&
            lhs.pitch == rhs.pitch &&
            lhs.yaw == rhs.yaw);
  }

  const Pose& PoseProcessor::getCurrentPose() const {
    if (pose_history_.empty()) {
      throw std::runtime_error("No poses in history");
    }
    return pose_history_.back();
  }

  bool PoseProcessor::isPoseOutOfBounds() const {

    Pose current_pose = getCurrentPose();

    return ((current_pose.x < bounding_box_.x_bounds.first) || (current_pose.x > bounding_box_.x_bounds.second)) || 
            ((current_pose.y < bounding_box_.y_bounds.first) || (current_pose.y > bounding_box_.y_bounds.second)) ||
            ((current_pose.z < bounding_box_.z_bounds.first) || (current_pose.z > bounding_box_.z_bounds.second));
  }

  bool PoseProcessor::isPoseOutofEllipse() const {

    if (control_ellipse_.semi_axes.first == 0.0 || control_ellipse_.semi_axes.second == 0.0) {
      throw std::runtime_error("Ellipse semi-axes must be non-zero");
    }

    Pose current_pose = getCurrentPose();

    double normalized_x = (current_pose.x - control_ellipse_.center.first) / control_ellipse_.semi_axes.first;
    double normalized_y = (current_pose.y - control_ellipse_.center.second) / control_ellipse_.semi_axes.second;

    return (pow(normalized_x, 2) + pow(normalized_y, 2) > 1.0);
  }

  void PoseProcessor::addPose(Pose new_pose) {

    Pose previous_pose = pose_history_.empty() ? Pose() : pose_history_.back();
    pose_history_.push_back(new_pose);

    if (pose_history_.size() > history_size_)
    {
      pose_history_.pop_front();
    }

    if (new_pose == previous_pose)
    {
      std::cout << new_pose.y;
      pose_identical_time_ = std::chrono::system_clock::now();
    }   
    else
    {
      pose_identical_time_.reset();
    }
  }

  bool PoseProcessor::hasLostTracking() const{
    // std::cout << std::chrono::duration<double>(*pose_identical_time_).count();
    if (!pose_identical_time_) {
      std::cout << "yo";
      return false;
    }
    std::cout << "ke";
    auto elapsed_time = std::chrono::duration<double>(std::chrono::system_clock::now() - *pose_identical_time_);
    return elapsed_time.count() > max_identical_seconds_;
  }
}