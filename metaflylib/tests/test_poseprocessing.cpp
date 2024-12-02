#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <metaflylib/poseprocessing.hpp>
#include <metaflylib/elements.hpp>
#include <stdexcept>
#include <chrono>
#include <thread>

using namespace metaflylib;
using Catch::Matchers::WithinAbs;

TEST_CASE("PoseProcessor Constructor Validation", "[PoseProcessor]") {
    Box valid_box{{0.0, 10.0}, {0.0, 10.0}, {0.0, 10.0}};
    Ellipse valid_ellipse{{5.0, 5.0}, {2.0, 3.0}};

    SECTION("Valid constructor parameters") {
        REQUIRE_NOTHROW(PoseProcessor(10, 1.0, valid_box, valid_ellipse));
    }

    SECTION("Zero history size") {
        REQUIRE_THROWS_AS(PoseProcessor(0, 1.0, valid_box, valid_ellipse), std::invalid_argument);
    }

    SECTION("Negative max identical seconds") {
        REQUIRE_THROWS_AS(PoseProcessor(10, -1.0, valid_box, valid_ellipse), std::invalid_argument);
    }
}

TEST_CASE("Adding and Retrieving Poses", "[PoseProcessor]") {
    Box box{{0.0, 10.0}, {0.0, 10.0}, {0.0, 10.0}};
    Ellipse ellipse{{5.0, 5.0}, {2.0, 3.0}};
    PoseProcessor processor(5, 1.0, box, ellipse);

    Pose pose1{1.0, 2.0, 3.0, Angle(0.0), Angle(0.0), Angle(0.0)};
    Pose pose2{4.0, 5.0, 6.0, Angle(0.0), Angle(0.0), Angle(0.0)};

    SECTION("Add and retrieve poses") {
        processor.addPose(pose1);
        REQUIRE(processor.getCurrentPose() == pose1);

        processor.addPose(pose2);
        REQUIRE(processor.getCurrentPose() == pose2);
    }

    SECTION("Adding poses beyond history size") {
        for (int i = 0; i < 10; ++i) {
            processor.addPose(pose1);
        }
        REQUIRE_NOTHROW(processor.getCurrentPose());
    }

    SECTION("Empty pose history throws exception") {
        PoseProcessor empty_processor(5, 1.0, box, ellipse);
        REQUIRE_THROWS_AS(empty_processor.getCurrentPose(), std::runtime_error);
    }
}

TEST_CASE("PoseProcessor Bounding Box Checks", "[PoseProcessor]") {
    Box box{{0.0, 10.0}, {0.0, 10.0}, {0.0, 10.0}};
    Ellipse ellipse{{5.0, 5.0}, {2.0, 3.0}};
    PoseProcessor processor(5, 1.0, box, ellipse);

    Pose inside_pose{5.0, 5.0, 5.0, Angle(0.0), Angle(0.0), Angle(0.0)};
    Pose outside_pose{15.0, 5.0, 5.0, Angle(0.0), Angle(0.0), Angle(0.0)};

    SECTION("Pose inside bounding box") {
        processor.addPose(inside_pose);
        REQUIRE_FALSE(processor.isPoseOutOfBounds());
    }

    SECTION("Pose outside bounding box") {
        processor.addPose(outside_pose);
        REQUIRE(processor.isPoseOutOfBounds());
    }
}

TEST_CASE("PoseProcessor Ellipse Checks", "[PoseProcessor]") {
    Box box{{0.0, 10.0}, {0.0, 10.0}, {0.0, 10.0}};
    Ellipse ellipse{{5.0, 5.0}, {2.0, 3.0}};
    PoseProcessor processor(5, 1.0, box, ellipse);

    Pose inside_pose{5.0, 5.0, 0.0, Angle(0.0), Angle(0.0), Angle(0.0)};
    Pose outside_pose{8.0, 5.0, 0.0, Angle(0.0), Angle(0.0), Angle(0.0)};
    Pose invalid_ellipse_pose{5.0, 5.0, 0.0, Angle(0.0), Angle(0.0), Angle(0.0)};

    SECTION("Pose inside ellipse") {
        processor.addPose(inside_pose);
        REQUIRE_FALSE(processor.isPoseOutofEllipse());
    }

    SECTION("Pose outside ellipse") {
        processor.addPose(outside_pose);
        REQUIRE(processor.isPoseOutofEllipse());
    }

    SECTION("Invalid ellipse semi-axes") {
        Ellipse invalid_ellipse{{5.0, 5.0}, {0.0, 0.0}};
        PoseProcessor invalid_processor(5, 1.0, box, invalid_ellipse);
        invalid_processor.addPose(invalid_ellipse_pose);
        REQUIRE_THROWS_AS(invalid_processor.isPoseOutofEllipse(), std::runtime_error);
    }
}

TEST_CASE("PoseProcessor Tracking Loss", "[PoseProcessor]") {
    Box box{{0.0, 10.0}, {0.0, 10.0}, {0.0, 10.0}};
    Ellipse ellipse{{5.0, 5.0}, {2.0, 3.0}};
    PoseProcessor processor(5, 1.0, box, ellipse);

    Pose pose{5.0, 5.0, 5.0, Angle(0.0), Angle(0.0), Angle(0.0)};

    SECTION("No tracking loss initially") {
        processor.addPose(pose);
        REQUIRE_FALSE(processor.hasLostTracking());
    }

    SECTION("Tracking loss after identical pose") {
        processor.addPose(pose);
        processor.addPose(pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500)); // Exceed 1 second threshold

        REQUIRE(processor.hasLostTracking());
    }

    SECTION("No tracking loss with pose change") {
        processor.addPose(pose);
        Pose new_pose{5.5, 5.0, 5.0, Angle(0.0), Angle(0.0), Angle(0.0)};
        processor.addPose(new_pose);
        REQUIRE_FALSE(processor.hasLostTracking());
    }
}
