#include <cstdio>
#include <cmath>
#include <iostream>
#include "metaflylib/elements.hpp"

namespace metaflylib
{
    // double normalize_angle(double rad)
    // {
    //     // Normalize to range (-PI, PI].
    //     if (almost_equal(rad, -PI))
    //     {
    //         return PI;
    //     }
    //     return atan2(sin(rad), cos(rad));
    // }

    // SPEED

    // Setter with automatic clamping
    void Speed::setValue(int val) {
      value = std::clamp(val, MIN_SPEED, MAX_SPEED);
    }

    // Overload assignment operator for convenience
    Speed& Speed::operator=(int val) {
      setValue(val);
      return *this;
    }

    // Arithmetic assignment operators (e.g., +=, -=)
    Speed& Speed::operator+=(int rhs) {
      setValue(value + rhs);
      return *this;
    }

    Speed& Speed::operator-=(int rhs) {
      setValue(value - rhs);
      return *this;
    }

    Speed& Speed::operator*=(int rhs) {
      setValue(value * rhs);
      return *this;
    }

    Speed& Speed::operator/=(int rhs) {
      if (rhs != 0) setValue(value / rhs);
      return *this;
    }

    // Overload binary arithmetic operators (e.g., +, -, *, /)
    // Plus
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

    // Minus
    Speed operator-(Speed lhs, int rhs) {
      lhs -= rhs;
      return lhs;
    }

    Speed operator-(int lhs, Speed rhs) {
      int result = lhs - rhs.value;
      return Speed(result);
    }

    Speed operator-(Speed lhs, Speed rhs) {
      int result = lhs.value - rhs.value;
      return Speed(result);
    }

    // Multiply
    Speed operator*(Speed lhs, int rhs) {
      lhs *= rhs;
      return lhs;
    }

    Speed operator*(int lhs, Speed rhs) {
      rhs *= lhs;
      return rhs;
    }

    // Divide
    Speed operator/(Speed lhs, int rhs) {
      if (rhs != 0) lhs /= rhs;
      return lhs;
    }

    // STEERING

    // Setter with automatic clamping
    void Steering::setValue(int val) {
      value = std::clamp(val, MIN_STEERING, MAX_STEERING);
    }

    // Overload assignment operator for convenience
    Steering& Steering::operator=(int val) {
      setValue(val);
      return *this;
    }

    // Arithmetic assignment operators (e.g., +=, -=)
    Steering& Steering::operator+=(int rhs) {
      setValue(value + rhs);
      return *this;
    }

    Steering& Steering::operator-=(int rhs) {
      setValue(value - rhs);
      return *this;
    }

    Steering& Steering::operator*=(int rhs) {
      setValue(value * rhs);
      return *this;
    }

    Steering& Steering::operator/=(int rhs) {
      if (rhs != 0) setValue(value / rhs);
      return *this;
    }

    // Overload binary arithmetic operators (e.g., +, -, *, /)
    // Plus
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

    // Minus
    Steering operator-(Steering lhs, int rhs) {
      lhs -= rhs;
      return lhs;
    }

    Steering operator-(int lhs, Steering rhs) {
      int result = lhs - rhs.value;
      return Steering(result);
    }

    Steering operator-(Steering lhs, Steering rhs) {
      int result = lhs.value - rhs.value;
      return Steering(result);
    }

    // Multiply
    Steering operator*(Steering lhs, int rhs) {
      lhs *= rhs;
      return lhs;
    }

    Steering operator*(int lhs, Steering rhs) {
      rhs *= lhs;
      return rhs;
    }

    // Divide
    Steering operator/(Steering lhs, int rhs) {
      if (rhs != 0) lhs /= rhs;
      return lhs;
    }

    // ANGLE

    double Angle::normalizeAngle(double value)
    {
      // Normalize to range (-PI, PI].
      if (almost_equal(value, -PI))
      {
        return PI;
      }
      return atan2(sin(value), cos(value));
    }

    // Setter with automatic clamping
    void Angle::setValue(double val) {
      value = normalizeAngle(val);
    }

    // Overload assignment operator for convenience
    Angle& Angle::operator=(double val) {
      setValue(val);
      return *this;
    }

    // Arithmetic assignment operators (e.g., +=, -=)
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

    // Overload binary arithmetic operators (e.g., +, -, *, /)
    // Plus
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

    // Minus
    Angle operator-(Angle lhs, double rhs) {
      lhs -= rhs;
      return lhs;
    }

    Angle operator-(double lhs, Angle rhs) {
      double result = lhs - rhs.value;
      return Angle(result);
    }

    Angle operator-(Angle lhs, Angle rhs) {
      double result = lhs.value - rhs.value;
      return Angle(result);
    }

    // Multiply
    Angle operator*(Angle lhs, double rhs) {
      lhs *= rhs;
      return lhs;
    }

    Angle operator*(double lhs, Angle rhs) {
      rhs *= lhs;
      return rhs;
    }

    // Divide
    Angle operator/(Angle lhs, double rhs) {
      if (rhs != 0) lhs /= rhs;
      return lhs;
    }

    // Vector2D operator-(const Point2D & head, const Point2D & tail)
    // {   
    //     return Vector2D{head.x - tail.x, head.y - tail.y};
    // }

    // Point2D operator+(const Point2D & tail, const Vector2D & disp)
    // {   
    //     return Point2D{tail.x + disp.x, tail.y + disp.y};
    // }

    // std::ostream & operator<<(std::ostream & os, const Point2D & p)
    // {
    //     return os << "[" << p.x << " " << p.y << "]";
    // }

    // std::istream & operator>>(std::istream & is, Point2D & p)
    // {
    //     const auto c = is.peek();        // examine the next character without extracting it

    //     if (c == '[') {
    //         is.get();         // remove the '[' character from the stream
    //         is >> p.x;
    //         is >> p.y;
    //         is.get();         // remove the ']' character from the stream
    //     } else {
    //         is >> p.x >> p.y;
    //     }

    //     is.ignore(100, '\n');
    //     return is;
    // }

    // std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    // {
    //     return os << "[" << v.x << " " << v.y << "]";
    // }

    // std::istream & operator>>(std::istream & is, Vector2D & v)
    // {
    //     const auto c = is.peek();        // examine the next character without extracting it

    //     if (c == '[') {
    //         is.get();         // remove the '[' character from the stream
    //         is >> v.x;
    //         is >> v.y;
    //         is.get();         // remove the ']' character from the stream
    //     } else {
    //         is >> v.x >> v.y;
    //     }

    //     is.ignore(100, '\n');
    //     return is;
    // }

    // Vector2D normalizeVector(const Vector2D & v)
    // {
    //     Vector2D v_hat{};
    //     double v_norm = sqrt(v.x * v.x + v.y * v.y);

    //     if(v_norm == 0.0)
    //     {
    //         std::cout << "INVALID VECTOR." << std::endl;
    //     }
    //     else
    //     {
    //         v_hat.x = v.x / v_norm;
    //         v_hat.y = v.y / v_norm;
    //     }

    //     return v_hat;
    // }

    // Vector2D operator-(const Vector2D & va, const Vector2D & vb)
    // {
    //     return Vector2D{va.x - vb.x, va.y - vb.y};
    // }

    // Vector2D operator+(const Vector2D & va, const Vector2D & vb)
    // {
    //     return Vector2D{va.x + vb.x, va.y + vb.y};
    // }

    // Vector2D operator*(const double & scale, const Vector2D & v)
    // {
    //     return Vector2D{scale * v.x, scale * v.y};
    // }

    // Vector2D operator*(const Vector2D & v, const double & scale)
    // {
    //     return Vector2D{scale * v.x, scale * v.y};
    // }

    // Vector2D operator-=(Vector2D & lhv, const Vector2D & rhv)
    // {
    //     lhv = lhv - rhv;
    //     return lhv;
    // }

    // Vector2D operator+=(Vector2D & lhv, const Vector2D & rhv)
    // {
    //     lhv = lhv + rhv;
    //     return lhv;
    // }

    // Vector2D operator*=(Vector2D & lhv, const double & scale)
    // {
    //     lhv = scale * scale * lhv;
    //     lhv = lhv * (1/scale);
    //     return lhv;
    // }

    // double dot(const Vector2D & va, const Vector2D & vb)
    // {
    //     return va.x * vb.x + va.y * vb.y;
    // }

    // double magnitude(const Vector2D & v)
    // {
    //     return sqrt(dot(v,v));
    // }

    // double angle(const Vector2D & va, const Vector2D & vb)
    // {
    //     double angle_a = 0.0, angle_b = 0.0;
        
    //     // Check first vector's edge case for atan2
    //     if(va.x == -0.0 && (va.y == 0.0 || va.y == -0.0))
    //     {
    //         angle_a = atan2(0.0,0.0);
    //     }
    //     else
    //     {
    //         angle_a = atan2(va.y, va.x);
    //     }

    //     // Check second vector's edge case for atan2
    //     if(vb.x == -0.0 && (vb.y == 0.0 || vb.y == -0.0))
    //     {
    //         angle_b = atan2(0.0,0.0);
    //     }
    //     else
    //     {
    //         angle_b = atan2(vb.y, vb.x);
    //     }
        
    //     return normalize_angle(angle_a - angle_b);
    // }
}