#ifndef METAFLYLIB_POSEPROCESSING_HPP_INCLUDE_GUARD
#define METAFLYLIB_POSEPROCESSING_HPP_INCLUDE_GUARD
/// \file
/// \brief Two-dimensional geometric primitives.

#include <chrono>
#include <algorithm>
#include <deque>
#include <cmath>
#include <optional>
#include <metaflylib/elements.hpp>
#include <iosfwd> // contains forward definitions for iostream objects
namespace metaflylib
{
  struct Pose {
    double x{0.0};
    double y{0.0}; 
    double z{0.0};
    Angle roll;
    Angle pitch;
    Angle yaw;
  };

  bool operator==(const Pose & lhs, const Pose & rhs);

  struct Box {
    std::pair<double, double> x_bounds{0.0, 0.0};
    std::pair<double, double> y_bounds{0.0, 0.0};
    std::pair<double, double> z_bounds{0.0, 0.0};
  };

  struct Ellipse {
    std::pair<double, double> center{0.0, 0.0};
    std::pair<double, double> semi_axes{0.0, 0.0};
  };

  class PoseProcessor {
    private:
      size_t history_size_{0};
      std::deque<Pose> pose_history_;
      std::optional<Timestamp> pose_identical_time_{std::nullopt};
      double max_identical_seconds_{1.0};

      Box bounding_box_;

      Ellipse control_ellipse_;
    
    public:
      PoseProcessor(size_t history_size, double max_identical_seconds_, Box bounding_box, Ellipse control_ellipse);

      const Pose& getCurrentPose() const;
      std::optional<Timestamp> getPoseIdenticalTime() const {return pose_identical_time_;}
      void addPose(Pose new_pose);

      bool isPoseOutOfBounds() const;
      bool isPoseOutofEllipse() const;
      bool hasLostTracking() const;
  };
}

#endif