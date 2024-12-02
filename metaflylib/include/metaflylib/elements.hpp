#ifndef METAFLYLIB_ELEMENTS_HPP_INCLUDE_GUARD
#define METAFLYLIB_ELEMENTS_HPP_INCLUDE_GUARD
/// \file
/// \brief Core library for geometric primitives and essential components for Metafly.

#include <chrono>
#include <algorithm>
#include <iosfwd> // Contains forward definitions for iostream objects

namespace metaflylib {

    /// \brief Mathematical constant for PI. Not in C++ standard until C++20.
    constexpr double PI = 3.14159265358979323846;

    /// \brief Compare two floating-point numbers for approximate equality.
    /// \param d1 - The first number to compare.
    /// \param d2 - The second number to compare.
    /// \param epsilon - Absolute threshold for equality.
    /// \return True if |d1 - d2| < epsilon.
    constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-6) {
        return std::abs(d1 - d2) < epsilon;
    }

    /// \brief Compile-time assertions to validate functionality.
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(2, 2.0), "can_typecast failed");

    /// \brief Timestamp type for tracking time points.
    using Timestamp = std::chrono::time_point<std::chrono::system_clock>;

    /// \class Speed
    /// \brief Represents a speed value clamped to a specified range [0, 127].
    class Speed {
    private:
        int value;
        static constexpr int MIN_SPEED = 0;
        static constexpr int MAX_SPEED = 127;

    public:
        /// \brief Constructor with clamping.
        /// \param val - Initial speed value.
        Speed(int val = 0) : value(std::clamp(val, MIN_SPEED, MAX_SPEED)) {}

        /// \brief Get the current speed value.
        /// \return The clamped speed value.
        int getValue() const { return value; }

        /// \brief Implicit conversion to int.
        operator int() const { return value; }

        /// \brief Set the speed value with clamping.
        /// \param val - The new speed value.
        void setValue(int val);

        // Overload assignment and arithmetic operators
        Speed& operator=(int val);
        Speed& operator+=(int rhs);
        Speed& operator-=(int rhs);
        Speed& operator*=(int rhs);
        Speed& operator/=(int rhs);

        // Binary arithmetic operators
        friend Speed operator+(Speed lhs, int rhs);
        friend Speed operator+(int lhs, Speed rhs);
        friend Speed operator+(Speed lhs, Speed rhs);

        friend Speed operator-(Speed lhs, int rhs);
        friend Speed operator-(int lhs, Speed rhs);
        friend Speed operator-(Speed lhs, Speed rhs);

        friend Speed operator*(Speed lhs, int rhs);
        friend Speed operator*(int lhs, Speed rhs);

        friend Speed operator/(Speed lhs, int rhs);

        // Comparison operators
        bool operator==(const Speed& other) const { return value == other.value; }
        bool operator!=(const Speed& other) const { return !(*this == other); }
        bool operator<(const Speed& other) const { return value < other.value; }
        bool operator<=(const Speed& other) const { return value <= other.value; }
        bool operator>(const Speed& other) const { return value > other.value; }
        bool operator>=(const Speed& other) const { return value >= other.value; }
    };

    /// \class Steering
    /// \brief Represents a steering value clamped to a range [-127, 127].
    class Steering {
    private:
        int value;
        static constexpr int MIN_STEERING = -127;
        static constexpr int MAX_STEERING = 127;

    public:
        /// \brief Constructor with clamping.
        /// \param val - Initial steering value.
        Steering(int val = 0) : value(std::clamp(val, MIN_STEERING, MAX_STEERING)) {}

        /// \brief Get the current steering value.
        /// \return The clamped steering value.
        int getValue() const { return value; }

        /// \brief Implicit conversion to int.
        operator int() const { return value; }

        /// \brief Set the steering value with clamping.
        /// \param val - The new steering value.
        void setValue(int val);

        // Overload assignment and arithmetic operators
        Steering& operator=(int val);
        Steering& operator+=(int rhs);
        Steering& operator-=(int rhs);
        Steering& operator*=(int rhs);
        Steering& operator/=(int rhs);

        // Binary arithmetic operators
        friend Steering operator+(Steering lhs, int rhs);
        friend Steering operator+(int lhs, Steering rhs);
        friend Steering operator+(Steering lhs, Steering rhs);

        friend Steering operator-(Steering lhs, int rhs);
        friend Steering operator-(int lhs, Steering rhs);
        friend Steering operator-(Steering lhs, Steering rhs);

        friend Steering operator*(Steering lhs, int rhs);
        friend Steering operator*(int lhs, Steering rhs);

        friend Steering operator/(Steering lhs, int rhs);

        // Comparison operators
        bool operator==(const Steering& other) const { return value == other.value; }
        bool operator!=(const Steering& other) const { return !(*this == other); }
        bool operator<(const Steering& other) const { return value < other.value; }
        bool operator<=(const Steering& other) const { return value <= other.value; }
        bool operator>(const Steering& other) const { return value > other.value; }
        bool operator>=(const Steering& other) const { return value >= other.value; }
    };

    /// \class Angle
    /// \brief Represents an angle value normalized to the range (-PI, PI].
    class Angle {
    private:
        double value;

        /// \brief Normalize an angle to the range (-PI, PI].
        /// \param value - The input angle in radians.
        /// \return The normalized angle.
        double normalizeAngle(double value);

    public:
        /// \brief Constructor with normalization.
        /// \param val - Initial angle in radians.
        Angle(double val = 0.0) : value(normalizeAngle(val)) {}

        /// \brief Get the current angle value.
        /// \return The normalized angle.
        double getValue() const { return value; }

        /// \brief Implicit conversion to double.
        operator double() const { return value; }

        /// \brief Set the angle value with normalization.
        /// \param val - The new angle in radians.
        void setValue(double val);

        // Overload assignment and arithmetic operators
        Angle& operator=(double val);
        Angle& operator+=(double rhs);
        Angle& operator-=(double rhs);
        Angle& operator*=(double rhs);
        Angle& operator/=(double rhs);

        // Binary arithmetic operators
        friend Angle operator+(Angle lhs, double rhs);
        friend Angle operator+(double lhs, Angle rhs);
        friend Angle operator+(Angle lhs, Angle rhs);

        friend Angle operator-(Angle lhs, double rhs);
        friend Angle operator-(double lhs, Angle rhs);
        friend Angle operator-(Angle lhs, Angle rhs);

        friend Angle operator*(Angle lhs, double rhs);
        friend Angle operator*(double lhs, Angle rhs);

        friend Angle operator/(Angle lhs, double rhs);

        // Comparison operators
        bool operator==(const Angle& other) const { return almost_equal(value, other.value); }
        bool operator!=(const Angle& other) const { return !(*this == other); }
        bool operator<(const Angle& other) const { return value < other.value; }
        bool operator<=(const Angle& other) const { return value <= other.value; }
        bool operator>(const Angle& other) const { return value > other.value; }
        bool operator>=(const Angle& other) const { return value >= other.value; }
    };

    /// \struct Controls
    /// \brief Combines Speed and Steering into a single control structure.
    struct Controls {
        Speed speed;      ///< Speed control component.
        Steering steering; ///< Steering control component.
    };
}

#endif
