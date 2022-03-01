#if !defined(_PUZZLE_UTILS_H_)
#define _PUZZLE_UTILS_H_

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <stdlib.h>

using ignition::math::Angle;
using ignition::math::Quaterniond;
using ignition::math::Vector3d;

/**
 *
 **/
inline float degToRad(float deg) { return IGN_PI / deg; }

class Utils {
private:
public:
  Utils() {}
  ~Utils() {}

  static Quaterniond lookAt(Vector3d from, Vector3d to,
                            Vector3d forward = Vector3d::UnitX) {
    Vector3d newDir = (to - from).Normalized();
    double dot = forward.Dot(newDir);

    if (abs(dot - (-1.0f)) < 0.000001f)
      return Quaterniond(forward, degToRad(180.0f));

    if (abs(dot - (1.0f)) < 0.000001f)
      return Quaterniond::Identity;

    float rotAngle = (float)acos(dot);
    Vector3d rosAxis = forward.Cross(newDir).Normalize();
    return Quaterniond(rosAxis, rotAngle);
  }
};

#endif // _PUZZLE_UTILS_H_
