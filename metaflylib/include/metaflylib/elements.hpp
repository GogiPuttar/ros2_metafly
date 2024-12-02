#ifndef METAFLYLIB_ELEMENTS_HPP_INCLUDE_GUARD
#define METAFLYLIB_ELEMENTS_HPP_INCLUDE_GUARD
/// \file
/// \brief Two-dimensional geometric primitives.

#include <chrono>
#include <algorithm>
#include <iosfwd> // contains forward definitions for iostream objects
namespace metaflylib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-6)
    {
        return std::abs(d1 - d2) < epsilon;
    }

    /// \brief wrap an angle to (-PI, PI]
    /// \param rad (angle in radians)
    /// \return an angle equivalent to rad but in the range (-PI, PI]
    // double normalize_angle(double rad);

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(2, 2.0), "can_typecast failed");

    using Timestamp = std::chrono::time_point<std::chrono::system_clock>;

    // Speed class
    class Speed {
      private:
        int value;
        static constexpr int MIN_SPEED = 0;
        static constexpr int MAX_SPEED = 127;

      public:
        // Constructor to initialize the value with clamping
        Speed(int val = 0) : value(std::clamp(val, MIN_SPEED, MAX_SPEED)) {}

        // Getter for the value
        int getValue() const { return value; }

        // Implicit conversion operator
        operator int() const { return value; }

        // Setter with automatic clamping
        void setValue(int val);

        // Overload assignment operator for convenience
        Speed& operator=(int val);

        // Arithmetic assignment operators (e.g., +=, -=)
        Speed& operator+=(int rhs);

        Speed& operator-=(int rhs);

        Speed& operator*=(int rhs);

        Speed& operator/=(int rhs);

        // Overload binary arithmetic operators (e.g., +, -, *, /)
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

    // Steering class
    class Steering {
      private:
        int value;
        static constexpr int MIN_STEERING = -127;
        static constexpr int MAX_STEERING = 127;

      public:
        // Constructor to initialize the value with clamping
        Steering(int val = 0) : value(std::clamp(val, MIN_STEERING, MAX_STEERING)) {}

        // Getter for the value
        int getValue() const { return value; }

        // Implicit conversion operator
        operator int() const { return value; }

        // Setter with automatic clamping
        void setValue(int val);

        // Overload assignment operator for convenience
        Steering& operator=(int val);

        // Arithmetic assignment operators (e.g., +=, -=)
        Steering& operator+=(int rhs);

        Steering& operator-=(int rhs);

        Steering& operator*=(int rhs);

        Steering& operator/=(int rhs);

        // Overload binary arithmetic operators (e.g., +, -, *, /)
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

    class Angle {

      private:
        double value;

        double normalizeAngle (double value);

      public:

        Angle(double val = 0.0) : value(normalizeAngle(val)) {}

        double getValue() const {return value;}

        operator double() const {return value;}

        void setValue(double val);

        // Overload assignment operator for convenience
        Angle& operator=(double val);

        // Arithmetic assignment operators (e.g., +=, -=)
        Angle& operator+=(double rhs);

        Angle& operator-=(double rhs);

        Angle& operator*=(double rhs);

        Angle& operator/=(double rhs);

        // Overload binary arithmetic operators (e.g., +, -, *, /)
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

    struct Controls{
      Speed speed;
      Steering steering;
    };
  
    // /// \brief a 2-Dimensional Point
    // struct Point2D
    // {
    //     /// \brief the x coordinate
    //     double x = 0.0;

    //     /// \brief the y coordinate
    //     double y = 0.0;
    // };

    // /// \brief output a 2 dimensional point as [xcomponent ycomponent]
    // /// \param os - stream to output to
    // /// \param p - the point to print
    // std::ostream & operator<<(std::ostream & os, const Point2D & p);

    // /// \brief input a 2 dimensional point
    // ///   You should be able to read vectors entered as follows:
    // ///   [x y] or x y
    // /// \param is - stream from which to read
    // /// \param p [out] - output vector
    // /// HINT: See operator>> for Vector2D
    // std::istream & operator>>(std::istream & is, Point2D & p);

    // /// \brief A 2-Dimensional Vector
    // struct Vector2D
    // {
    //     /// \brief the x coordinate
    //     double x = 0.0;

    //     /// \brief the y coordinate
    //     double y = 0.0;
    // };

    // /// \brief Subtracting one point from another yields a vector
    // /// \param head point corresponding to the head of the vector
    // /// \param tail point corresponding to the tail of the vector
    // /// \return a vector that points from p1 to p2
    // /// NOTE: this is not implemented in terms of -= because subtracting two Point2D yields a Vector2D
    // Vector2D operator-(const Point2D & head, const Point2D & tail);

    // /// \brief Adding a vector to a point yields a new point displaced by the vector
    // /// \param tail The origin of the vector's tail
    // /// \param disp The displacement vector
    // /// \return the point reached by displacing by disp from tail
    // /// NOTE: this is not implemented in terms of += because of the different types
    // Point2D operator+(const Point2D & tail, const Vector2D & disp);

    // /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    // /// \param os - stream to output to
    // /// \param v - the vector to print
    // std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    // /// \brief input a 2 dimensional vector
    // ///   You should be able to read vectors entered as follows:
    // ///   [x y] or x y
    // /// \param is - stream from which to read
    // /// \param v [out] - output vector
    // /// Hint: The following may be useful:
    // /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    // /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    // ///
    // /// The way input works is (more or less): what the user types is stored in a buffer until the user enters
    // /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    // /// Typically, each character is examined and then removed from the buffer automatically.
    // /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    // /// an error flag is set on the stream object (e.g., std::cin).
    // ///
    // /// We have lower level control however. For example:
    // /// peek looks at the next unprocessed character in the buffer without removing it
    // /// get removes the next unprocessed character from the buffer.
    // std::istream & operator>>(std::istream & is, Vector2D & v);

    // // Normalize Vector
    // Vector2D normalizeVector(const Vector2D & v);

    // /// \brief Subtracting two vectors yields a new vector
    // /// \param va Vector A
    // /// \param vb Vector B
    // /// \return Vector A relative to vector B
    // Vector2D operator-(const Vector2D & va, const Vector2D & vb);

    // /// \brief Adding two vectors yields a new vector
    // /// \param va Vector A
    // /// \param vb Vector B
    // /// \return Resultant vector
    // Vector2D operator+(const Vector2D & va, const Vector2D & vb);

    // /// \brief Scaling a vector
    // /// \param scale scalar
    // /// \param v Vector to be scaled
    // /// \return Scaled vector
    // Vector2D operator*(const double & scale, const Vector2D & v);

    // /// \brief Scaling a vector
    // /// \param v Vector to be scaled
    // /// \param scale scalar
    // /// \return Scaled vector
    // Vector2D operator*(const Vector2D & v, const double & scale);

    // /// \brief Subtracting a vector from a vector, changes the vector
    // /// \param rhv Right Hand Vector
    // /// \return Diminished vector
    // Vector2D operator-=(Vector2D & lhv, const Vector2D & rhv);

    // /// \brief Adding a vector to a vector, changes the vector
    // /// \param rhv Right Hand Vector
    // /// \return Extended vector
    // Vector2D operator+=(Vector2D & lhv, const Vector2D & rhv);    

    // /// \brief Multiplying a vector with a scalar, scales the vector
    // /// \param rhv Right Hand Vector
    // /// \return Scaled vector
    // Vector2D operator*=(Vector2D & lhv, const double & scale);   

    // /// \brief Dot product of two vectors yields a scalar
    // /// \param va Vector A
    // /// \param vb Vector B
    // /// \return Dot product
    // double dot(const Vector2D & va, const Vector2D & vb);

    // /// \brief Magnitude of a vector is a non-negative scalar
    // /// \param v Vector 
    // /// \return Magnitude
    // double magnitude(const Vector2D & v);

    // /// \brief Compute the angle between two vectors
    // /// \param va Vector A
    // /// \param vb Vector B
    // /// \return Angle(V_a) - Angle(V_b) (positive counterclockwise) 
    // double angle(const Vector2D & va, const Vector2D & vb);
}

#endif