#include <cstdio>
#include <cmath>
#include <iostream>
#include "metaflylib/elements.hpp"
#include "metaflylib/poseprocessing.hpp"

namespace metaflylib
{
  /// \brief Constructs a PoseProcessor with specified parameters.
  /// \param history_size - The maximum size of the pose history.
  /// \param max_identical_seconds - The maximum duration for identical poses before tracking is considered lost.
  /// \param bounding_box - The bounding box for checking out-of-bounds poses.
  /// \param control_ellipse - The ellipse for checking poses in the XY-plane.
  /// \throws std::invalid_argument if `history_size` or `max_identical_seconds` is less than or equal to 0.
  PoseProcessor::PoseProcessor(size_t history_size, double max_identical_seconds, Box bounding_box, Ellipse control_ellipse) 
  : history_size_(history_size), max_identical_seconds_(max_identical_seconds), bounding_box_(bounding_box), control_ellipse_(control_ellipse) {
    if (history_size_ <= 0) {
      throw std::invalid_argument("History size must be greater than 0");
    }
    if (max_identical_seconds_ <= 0) {
      throw std::invalid_argument("Max identical seconds must be greater than 0");
    }
  }

  /// \brief Equality operator for Pose objects.
  /// \param lhs - The first pose to compare.
  /// \param rhs - The second pose to compare.
  /// \return True if all components of the poses are equal, otherwise false.
  bool operator==(const Pose &lhs, const Pose &rhs) {
    return (almost_equal(lhs.x, rhs.x) &&
            almost_equal(lhs.y, rhs.y) &&
            almost_equal(lhs.z, rhs.z) &&
            lhs.roll == rhs.roll &&
            lhs.pitch == rhs.pitch &&
            lhs.yaw == rhs.yaw);
  }

  /// \brief Retrieves the most recent pose from the pose history.
  /// \return A reference to the most recent pose.
  /// \throws std::runtime_error if the pose history is empty.
  const Pose& PoseProcessor::getCurrentPose() const {
    if (pose_history_.empty()) {
      throw std::runtime_error("No poses in history");
    }
    return pose_history_.back();
  }

  /// \brief Checks if the current pose is outside the defined bounding box.
  /// \return True if the current pose is out of bounds, otherwise false.
  bool PoseProcessor::isPoseOutOfBounds() const {
    Pose current_pose = getCurrentPose();

    return ((current_pose.x < bounding_box_.x_bounds.first) || (current_pose.x > bounding_box_.x_bounds.second)) || 
           ((current_pose.y < bounding_box_.y_bounds.first) || (current_pose.y > bounding_box_.y_bounds.second)) ||
           ((current_pose.z < bounding_box_.z_bounds.first) || (current_pose.z > bounding_box_.z_bounds.second));
  }

  /// \brief Checks if the current pose is outside the defined ellipse in the XY-plane.
  /// \return True if the current pose is outside the ellipse, otherwise false.
  /// \throws std::runtime_error if the ellipse semi-axes are zero.
  bool PoseProcessor::isPoseOutofEllipse() const {
    if (control_ellipse_.semi_axes.first == 0.0 || control_ellipse_.semi_axes.second == 0.0) {
      throw std::runtime_error("Ellipse semi-axes must be non-zero");
    }

    Pose current_pose = getCurrentPose();

    double normalized_x = (current_pose.x - control_ellipse_.center.first) / control_ellipse_.semi_axes.first;
    double normalized_y = (current_pose.y - control_ellipse_.center.second) / control_ellipse_.semi_axes.second;

    return (pow(normalized_x, 2) + pow(normalized_y, 2) > 1.0);
  }

  /// \brief Adds a new pose to the history and updates tracking state.
  /// \param new_pose - The new pose to add.
  /// If the new pose is identical to the previous pose, updates the timestamp of identical poses.
  void PoseProcessor::addPose(Pose new_pose) {
    Pose previous_pose = pose_history_.empty() ? Pose() : pose_history_.back();
    pose_history_.push_back(new_pose);

    if (pose_history_.size() > history_size_) {
      pose_history_.pop_front();
    }

    if (new_pose == previous_pose) {
      pose_identical_time_ = std::chrono::system_clock::now();
    } else {
      pose_identical_time_.reset();
    }
  }

  /// \brief Checks if tracking has been lost due to consecutive identical poses.
  /// \return True if the elapsed time for identical poses exceeds the maximum allowed duration, otherwise false.
  bool PoseProcessor::hasLostTracking() const {
    if (!pose_identical_time_) {
      return false;
    }
    auto elapsed_time = std::chrono::duration<double>(std::chrono::system_clock::now() - *pose_identical_time_);
    return elapsed_time.count() > max_identical_seconds_;
  }
}
