#include <sstream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "metaflylib/elements.hpp"

using metaflylib::Speed;

TEST_CASE("Speed clamping works", "[Speed]") {
    Speed s1(-50); // Below minimum
    Speed s2(50);  // Within range
    Speed s3(200); // Above maximum

    REQUIRE(s1.getValue() == 0);   // Clamped to minimum
    REQUIRE(s2.getValue() == 50); // Unchanged
    REQUIRE(s3.getValue() == 127); // Clamped to maximum
}

TEST_CASE("Speed arithmetic assignment operators work", "[Speed]") {
    Speed s(50);

    s += 100; // Addition with clamping
    REQUIRE(s.getValue() == 127); // Clamped to maximum

    s -= 200; // Subtraction with clamping
    REQUIRE(s.getValue() == 0);   // Clamped to minimum

    s = 50;    // Reset to a mid-value
    s *= 3;    // Multiplication with clamping
    REQUIRE(s.getValue() == 127); // Clamped to maximum

    s /= 0;    // Division by zero (should do nothing)
    REQUIRE(s.getValue() == 127);

    s /= 2;    // Valid division
    REQUIRE(s.getValue() == 63);
}

TEST_CASE("Speed binary arithmetic operators work", "[Speed]") {
    Speed s1(50), s2(30);

    Speed s3 = s1 + 100; // Addition with clamping
    REQUIRE(s3.getValue() == 127);

    Speed s4 = 100 + s1; // Addition with clamping (commutative)
    REQUIRE(s4.getValue() == 127);

    Speed s5 = s1 + s2; // Adding two Speed objects
    REQUIRE(s5.getValue() == 80);

    Speed s6 = s1 - 60; // Subtraction
    REQUIRE(s6.getValue() == 0);

    Speed s7 = 60 - s1; // Subtraction (commutative)
    REQUIRE(s7.getValue() == 10);

    Speed s8 = s1 - s2; // Subtracting two Speed objects
    REQUIRE(s8.getValue() == 20);

    Speed s9 = s1 * 2; // Multiplication
    REQUIRE(s9.getValue() == 100);

    Speed s10 = 3 * s1; // Multiplication (commutative)
    REQUIRE(s10.getValue() == 127); // Clamped to maximum

    Speed s11 = s1 / 2; // Division
    REQUIRE(s11.getValue() == 25);
}

TEST_CASE("Speed comparison operators work", "[Speed]") {
    Speed s1(50), s2(30), s3(50);

    REQUIRE(s1 == s3);   // Equality
    REQUIRE(s1 != s2);   // Inequality
    REQUIRE(s2 < s1);    // Less than
    REQUIRE(s2 <= s1);   // Less than or equal
    REQUIRE(s1 > s2);    // Greater than
    REQUIRE(s1 >= s3);   // Greater than or equal
}

TEST_CASE("Speed assignment works", "[Speed]") {
    Speed s;
    s = 300; // Assignment with clamping
    REQUIRE(s.getValue() == 127);

    s = -50; // Assignment with clamping
    REQUIRE(s.getValue() == 0);

    s = 70; // Assignment within range
    REQUIRE(s.getValue() == 70);
}

TEST_CASE("Speed default constructor works", "[Speed]") {
    Speed s;
    REQUIRE(s.getValue() == 0); // Default value is 0
}


using metaflylib::Steering;

TEST_CASE("Steering clamping works", "[Steering]") {
    Steering s1(-200); // Below minimum
    Steering s2(50);   // Within range
    Steering s3(300);  // Above maximum

    REQUIRE(s1.getValue() == -127); // Clamped to minimum
    REQUIRE(s2.getValue() == 50);  // Unchanged
    REQUIRE(s3.getValue() == 127); // Clamped to maximum
}

TEST_CASE("Steering arithmetic assignment operators work", "[Steering]") {
    Steering s(50);

    s += 100; // Addition with clamping
    REQUIRE(s.getValue() == 127); // Clamped to maximum

    s -= 300; // Subtraction with clamping
    REQUIRE(s.getValue() == -127); // Clamped to minimum

    s = 50;    // Reset to a mid-value
    s *= 3;    // Multiplication with clamping
    REQUIRE(s.getValue() == 127); // Clamped to maximum

    s /= 0;    // Division by zero (should do nothing)
    REQUIRE(s.getValue() == 127);

    s /= 2;    // Valid division
    REQUIRE(s.getValue() == 63);
}

TEST_CASE("Steering binary arithmetic operators work", "[Steering]") {
    Steering s1(50), s2(-30);

    Steering s3 = s1 + 100; // Addition with clamping
    REQUIRE(s3.getValue() == 127);

    Steering s4 = 100 + s1; // Addition with clamping (commutative)
    REQUIRE(s4.getValue() == 127);

    Steering s5 = s1 + s2; // Adding two Steering objects
    REQUIRE(s5.getValue() == 20);

    Steering s6 = s1 - 60; // Subtraction
    REQUIRE(s6.getValue() == -10);

    Steering s7 = 60 - s1; // Subtraction (commutative)
    REQUIRE(s7.getValue() == 10);

    Steering s8 = s1 - s2; // Subtracting two Steering objects
    REQUIRE(s8.getValue() == 80);

    Steering s9 = s1 * 2; // Multiplication
    REQUIRE(s9.getValue() == 100);

    Steering s10 = 3 * s1; // Multiplication (commutative)
    REQUIRE(s10.getValue() == 127); // Clamped to maximum

    Steering s11 = s1 / 2; // Division
    REQUIRE(s11.getValue() == 25);
}

TEST_CASE("Steering comparison operators work", "[Steering]") {
    Steering s1(50), s2(-30), s3(50);

    REQUIRE(s1 == s3);   // Equality
    REQUIRE(s1 != s2);   // Inequality
    REQUIRE(s2 < s1);    // Less than
    REQUIRE(s2 <= s1);   // Less than or equal
    REQUIRE(s1 > s2);    // Greater than
    REQUIRE(s1 >= s3);   // Greater than or equal
}

TEST_CASE("Steering assignment works", "[Steering]") {
    Steering s;
    s = 300; // Assignment with clamping
    REQUIRE(s.getValue() == 127);

    s = -200; // Assignment with clamping
    REQUIRE(s.getValue() == -127);

    s = 70; // Assignment within range
    REQUIRE(s.getValue() == 70);
}

TEST_CASE("Steering default constructor works", "[Steering]") {
    Steering s;
    REQUIRE(s.getValue() == 0); // Default value is 0
}

using metaflylib::Angle;
using metaflylib::PI;
using Catch::Matchers::WithinAbs;

TEST_CASE("Angle normalization works", "[Angle]") {
    Angle a1(PI + 1.0);   // Outside range
    Angle a2(-PI - 1.0);  // Outside range
    Angle a3(PI / 2.0);   // Within range
    Angle a4(0);          // Zero

    REQUIRE_THAT(static_cast<double>(a1), WithinAbs(-PI + 1.0, 1.0e-6));
    REQUIRE_THAT(static_cast<double>(a2), WithinAbs(PI - 1.0, 1.0e-6));
    REQUIRE_THAT(static_cast<double>(a3), WithinAbs(PI / 2.0, 1.0e-6));
    REQUIRE_THAT(static_cast<double>(a4), WithinAbs(0.0, 1.0e-6));
}

TEST_CASE("Angle arithmetic works", "[Angle]") {
    Angle a1(PI / 4.0);  // 45 degrees
    Angle a2(PI / 2.0);  // 90 degrees

    REQUIRE_THAT(static_cast<double>(a1 + a2), WithinAbs(PI * 3.0 / 4.0, 1.0e-6));
    REQUIRE_THAT(static_cast<double>(a2 - a1), WithinAbs(PI / 4.0, 1.0e-6));
    REQUIRE_THAT(static_cast<double>(a1 * 2.0), WithinAbs(PI / 2.0, 1.0e-6));
    REQUIRE_THAT(static_cast<double>(a2 / 2.0), WithinAbs(PI / 4.0, 1.0e-6));
}

TEST_CASE("Angle assignment and comparison works", "[Angle]") {
    Angle a1(PI);
    Angle a2(-PI);

    REQUIRE(a1 == a2);  // PI and -PI are equivalent
    REQUIRE(static_cast<double>(a1) != 0);    // PI is not 0
}

TEST_CASE("Angle setters work correctly", "[Angle]") {
    Angle a;
    a.setValue(PI + 2.0);
    REQUIRE_THAT(static_cast<double>(a), WithinAbs(-PI + 2.0, 1.0e-6));
}

TEST_CASE("Angle implicit conversion works", "[Angle]") {
    Angle a(PI / 3.0);
    double d = a;
    REQUIRE_THAT(d, WithinAbs(PI / 3.0, 1.0e-6));
}