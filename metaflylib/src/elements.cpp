#include <cstdio>
#include <cmath>
#include <iostream>
#include "metaflylib/elements.hpp"

namespace metaflylib
{
    // SPEED

    /// \brief Sets the value of speed with automatic clamping to the valid range.
    /// \param val - The new speed value to set.
    void Speed::setValue(int val) {
      value = std::clamp(val, MIN_SPEED, MAX_SPEED);
    }

    /// \brief Assigns a new value to the Speed object with clamping.
    /// \param val - The value to assign.
    /// \return Reference to the modified Speed object.
    Speed& Speed::operator=(int val) {
      setValue(val);
      return *this;
    }

    /// \brief Adds a value to the speed.
    /// \param rhs - The value to add.
    /// \return Reference to the modified Speed object.
    Speed& Speed::operator+=(int rhs) {
      setValue(value + rhs);
      return *this;
    }

    /// \brief Subtracts a value from the speed.
    /// \param rhs - The value to subtract.
    /// \return Reference to the modified Speed object.
    Speed& Speed::operator-=(int rhs) {
      setValue(value - rhs);
      return *this;
    }

    /// \brief Multiplies the speed by a value.
    /// \param rhs - The multiplier.
    /// \return Reference to the modified Speed object.
    Speed& Speed::operator*=(int rhs) {
      setValue(value * rhs);
      return *this;
    }

    /// \brief Divides the speed by a value.
    /// \param rhs - The divisor (non-zero).
    /// \return Reference to the modified Speed object.
    Speed& Speed::operator/=(int rhs) {
      if (rhs != 0) setValue(value / rhs);
      return *this;
    }

    /// Binary arithmetic operators for Speed
    Speed operator+(Speed lhs, int rhs) {
      lhs += rhs;
      return lhs;
    }

    Speed operator+(int lhs, Speed rhs) {
      rhs += lhs;
      return rhs;
    }

    Speed operator+(Speed lhs, Speed rhs) {
      lhs += rhs.value;
      return lhs;
    }

    Speed operator-(Speed lhs, int rhs) {
      lhs -= rhs;
      return lhs;
    }

    Speed operator-(int lhs, Speed rhs) {
      return Speed(lhs - rhs.value);
    }

    Speed operator-(Speed lhs, Speed rhs) {
      return Speed(lhs.value - rhs.value);
    }

    Speed operator*(Speed lhs, int rhs) {
      lhs *= rhs;
      return lhs;
    }

    Speed operator*(int lhs, Speed rhs) {
      rhs *= lhs;
      return rhs;
    }

    Speed operator/(Speed lhs, int rhs) {
      if (rhs != 0) lhs /= rhs;
      return lhs;
    }

    // STEERING

    /// \brief Sets the value of steering with automatic clamping to the valid range.
    /// \param val - The new steering value to set.
    void Steering::setValue(int val) {
      value = std::clamp(val, MIN_STEERING, MAX_STEERING);
    }

    /// \brief Assigns a new value to the Steering object with clamping.
    /// \param val - The value to assign.
    /// \return Reference to the modified Steering object.
    Steering& Steering::operator=(int val) {
      setValue(val);
      return *this;
    }

    /// \brief Adds a value to the steering.
    /// \param rhs - The value to add.
    /// \return Reference to the modified Steering object.
    Steering& Steering::operator+=(int rhs) {
      setValue(value + rhs);
      return *this;
    }

    /// \brief Subtracts a value from the steering.
    /// \param rhs - The value to subtract.
    /// \return Reference to the modified Steering object.
    Steering& Steering::operator-=(int rhs) {
      setValue(value - rhs);
      return *this;
    }

    /// \brief Multiplies the steering by a value.
    /// \param rhs - The multiplier.
    /// \return Reference to the modified Steering object.
    Steering& Steering::operator*=(int rhs) {
      setValue(value * rhs);
      return *this;
    }

    /// \brief Divides the steering by a value.
    /// \param rhs - The divisor (non-zero).
    /// \return Reference to the modified Steering object.
    Steering& Steering::operator/=(int rhs) {
      if (rhs != 0) setValue(value / rhs);
      return *this;
    }

    /// Binary arithmetic operators for Steering
    Steering operator+(Steering lhs, int rhs) {
      lhs += rhs;
      return lhs;
    }

    Steering operator+(int lhs, Steering rhs) {
      rhs += lhs;
      return rhs;
    }

    Steering operator+(Steering lhs, Steering rhs) {
      lhs += rhs.value;
      return lhs;
    }

    Steering operator-(Steering lhs, int rhs) {
      lhs -= rhs;
      return lhs;
    }

    Steering operator-(int lhs, Steering rhs) {
      return Steering(lhs - rhs.value);
    }

    Steering operator-(Steering lhs, Steering rhs) {
      return Steering(lhs.value - rhs.value);
    }

    Steering operator*(Steering lhs, int rhs) {
      lhs *= rhs;
      return lhs;
    }

    Steering operator*(int lhs, Steering rhs) {
      rhs *= lhs;
      return rhs;
    }

    Steering operator/(Steering lhs, int rhs) {
      if (rhs != 0) lhs /= rhs;
      return lhs;
    }

    // ANGLE

    /// \brief Normalize an angle to the range (-PI, PI].
    /// \param value - The input angle in radians.
    /// \return The normalized angle in the range (-PI, PI].
    double Angle::normalizeAngle(double value) {
      if (almost_equal(value, -PI)) {
        return PI;
      }
      return atan2(sin(value), cos(value));
    }

    /// \brief Sets the angle value with automatic normalization.
    /// \param val - The new angle value in radians.
    void Angle::setValue(double val) {
      value = normalizeAngle(val);
    }

    /// \brief Assigns a new value to the Angle object with normalization.
    /// \param val - The value to assign.
    /// \return Reference to the modified Angle object.
    Angle& Angle::operator=(double val) {
      setValue(val);
      return *this;
    }

    /// Arithmetic assignment operators for Angle
    Angle& Angle::operator+=(double rhs) {
      setValue(value + rhs);
      return *this;
    }

    Angle& Angle::operator-=(double rhs) {
      setValue(value - rhs);
      return *this;
    }

    Angle& Angle::operator*=(double rhs) {
      setValue(value * rhs);
      return *this;
    }

    Angle& Angle::operator/=(double rhs) {
      if (rhs != 0) setValue(value / rhs);
      return *this;
    }

    /// Binary arithmetic operators for Angle
    Angle operator+(Angle lhs, double rhs) {
      lhs += rhs;
      return lhs;
    }

    Angle operator+(double lhs, Angle rhs) {
      rhs += lhs;
      return rhs;
    }

    Angle operator+(Angle lhs, Angle rhs) {
      lhs += rhs.value;
      return lhs;
    }

    Angle operator-(Angle lhs, double rhs) {
      lhs -= rhs;
      return lhs;
    }

    Angle operator-(double lhs, Angle rhs) {
      return Angle(lhs - rhs.value);
    }

    Angle operator-(Angle lhs, Angle rhs) {
      return Angle(lhs.value - rhs.value);
    }

    Angle operator*(Angle lhs, double rhs) {
      lhs *= rhs;
      return lhs;
    }

    Angle operator*(double lhs, Angle rhs) {
      rhs *= lhs;
      return rhs;
    }

    Angle operator/(Angle lhs, double rhs) {
      if (rhs != 0) lhs /= rhs;
      return lhs;
    }
}
