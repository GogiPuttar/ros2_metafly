#ifndef METAFLYLIB_POSEPROCESSING_HPP_INCLUDE_GUARD
#define METAFLYLIB_POSEPROCESSING_HPP_INCLUDE_GUARD
/// \file
/// \brief Pose processing utilities for geometric computations.

#include <chrono>
#include <algorithm>
#include <deque>
#include <cmath>
#include <optional>
#include <metaflylib/elements.hpp>
#include <iosfwd> // Contains forward declarations for iostream objects

namespace metaflylib
{
    /// \brief Represents a 3D pose with position and orientation.
    struct Pose {
        double x{0.0}; ///< X-coordinate of the pose.
        double y{0.0}; ///< Y-coordinate of the pose.
        double z{0.0}; ///< Z-coordinate of the pose.
        Angle roll;    ///< Roll angle (rotation about X-axis).
        Angle pitch;   ///< Pitch angle (rotation about Y-axis).
        Angle yaw;     ///< Yaw angle (rotation about Z-axis).
    };

    /// \brief Compares two poses for equality.
    /// \param lhs - The first pose.
    /// \param rhs - The second pose.
    /// \return True if all fields in the poses are equal, otherwise false.
    bool operator==(const Pose &lhs, const Pose &rhs);

    /// \brief Represents a 3D bounding box defined by axis-aligned constraints.
    struct Box {
        std::pair<double, double> x_bounds{0.0, 0.0}; ///< X-axis bounds (min, max).
        std::pair<double, double> y_bounds{0.0, 0.0}; ///< Y-axis bounds (min, max).
        std::pair<double, double> z_bounds{0.0, 0.0}; ///< Z-axis bounds (min, max).
    };

    /// \brief Represents an ellipse in the XY-plane.
    struct Ellipse {
        std::pair<double, double> center{0.0, 0.0};     ///< Center of the ellipse (x, y).
        std::pair<double, double> semi_axes{0.0, 0.0}; ///< Semi-major and semi-minor axes lengths.
    };

    /// \brief A processor for managing pose history and applying geometric constraints.
    class PoseProcessor {
    private:
        size_t history_size_{0}; ///< Maximum size of the pose history.
        std::deque<Pose> pose_history_; ///< History of recent poses.
        std::optional<Timestamp> pose_identical_time_{std::nullopt}; ///< Time when consecutive identical poses began.
        double max_identical_seconds_{1.0}; ///< Maximum allowed duration for identical poses.

        Box bounding_box_; ///< 3D bounding box to check pose constraints.
        Ellipse control_ellipse_; ///< Ellipse to check pose constraints in the XY-plane.

    public:
        /// \brief Constructs a PoseProcessor.
        /// \param history_size - Maximum number of poses to retain in the history.
        /// \param max_identical_seconds - Maximum duration allowed for identical poses.
        /// \param bounding_box - Bounding box for out-of-bounds checks.
        /// \param control_ellipse - Ellipse for in-ellipse checks.
        /// \throws std::invalid_argument if history_size is less than or equal to zero.
        /// \throws std::invalid_argument if max_identical_seconds is less than or equal to zero.
        PoseProcessor(size_t history_size, double max_identical_seconds, Box bounding_box, Ellipse control_ellipse);

        /// \brief Retrieves the most recent pose.
        /// \return A constant reference to the most recent pose.
        /// \throws std::runtime_error if the pose history is empty.
        const Pose& getCurrentPose() const;

        /// \brief Retrieves the timestamp when identical poses began.
        /// \return An optional timestamp indicating the start of identical poses.
        std::optional<Timestamp> getPoseIdenticalTime() const { return pose_identical_time_; }

        /// \brief Adds a new pose to the history.
        /// \param new_pose - The pose to add.
        /// If the pose is identical to the previous pose, updates the identical pose timestamp.
        void addPose(Pose new_pose);

        /// \brief Checks if the current pose is outside the bounding box.
        /// \return True if the current pose is out of bounds, otherwise false.
        bool isPoseOutOfBounds() const;

        /// \brief Checks if the current pose is outside the ellipse.
        /// \return True if the current pose is out of the ellipse, otherwise false.
        /// \throws std::runtime_error if the ellipse semi-axes are zero.
        bool isPoseOutofEllipse() const;

        /// \brief Checks if tracking has been lost due to identical poses.
        /// \return True if identical poses exceed the maximum allowed duration, otherwise false.
        bool hasLostTracking() const;
    };
}

#endif
