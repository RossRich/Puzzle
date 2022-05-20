#if !defined(_PUZZLE_LINE_H_)
#define _PUZZLE_LINE_H_

#include <tf2/LinearMath/Vector3.h>
#include <ros/ros.h>
#include <iostream>

class Line {
private:
  const tf2::Vector3 _point1;
  const tf2::Vector3 _point2;
  tf2::Vector3 _midpoint;

public:
  Line(const tf2::Vector3 &point3d1, const tf2::Vector3 &point3d2) : _point1(point3d1), _point2(point3d2) {
    _midpoint = tf2::lerp(_point1, _point2, .5f);
  }
  ~Line() {}

  bool operator==(Line &line) {
    return _point1 == line._point1 && _point2 == line._point2;
  }

  friend std::ostream& operator<<(std::ostream &stream, const Line &line) {
    stream << "\np1 x: " << line._point1.x() << " y: " << line._point1.y() << " z: " << line._point1.z();
    stream << "\np2 x: " << line._point2.x() << " y: " << line._point2.y() << " z: " << line._point2.z();
    stream << "\nmid x: " << line._midpoint.x() << " y: " << line._midpoint.y() << " z: " << line._midpoint.z(); 
    return stream;
  }

  inline const tf2::Vector3 &getP1() {
    return _point1;
  }

  inline const tf2::Vector3 &getP2() {
    return _point2;
  }

  inline const tf2::Vector3 &getMidpoint() {
    return _midpoint;
  }

  inline float length2() {
    return tf2::tf2Distance2(_point2, _point1);
  }

  inline float length() {
    return tf2::tf2Distance(_point2, _point1);
  }

  // S = ah
  float distToPoint2(const tf2::Vector3 &point) {
    float dist2ToP1 = tf2::tf2Distance2(point, _point1);
    float dist2ToP2 = tf2::tf2Distance2(point, _point2);

    auto nearPoint = std::ref(_point1);
    if (dist2ToP2 < dist2ToP1)
      nearPoint = std::ref(_point2);

    tf2::Vector3 lineDir = _point2 - _point1;
    if (lineDir.length2() == 0.f)
      return -1.f;

    tf2::Vector3 numerator = tf2::tf2Cross(point - nearPoint, lineDir); // vector length equals area of parallelogram

    return numerator.length2() / lineDir.length2();
  }

  float distToPoint(tf2::Vector3 &point) {
    float res = distToPoint2(point);
    return res == -1.f ? -1.f : sqrtf(res);
  }

  /**
   * https://gamedev.stackexchange.com/questions/72528/how-can-i-project-a-3d-point-onto-a-3d-line
   *
   * A + dot(AP,AB) / dot(AB,AB) * AB
   * A = line point1
   * B = line point2
   * P = incoming point
   */
  tf2::Vector3 porjectPoint(const tf2::Vector3 &point) {
    tf2::Vector3 AB = _point2 - _point1;
    tf2::Vector3 AP = point - _point1;

    tf2::Vector3 res = _point1;

    float numerator = tf2::tf2Dot(AP, AB);
    float denominator = tf2::tf2Dot(AB, AB);

    res = res + numerator / denominator * AB;

    return res;
  }
};

#endif // _PUZZLE_LINE_H_
