#ifndef ROBOT_UTILS_ROBOT_UTILS_H
#define ROBOT_UTILS_ROBOT_UTILS_H

#include <cmath>

namespace robot_utils
{

template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

double normalize_angle(double angle)
{
  return angle - round(angle / M_PI) * M_PI;
}

}

#endif // ROBOT_UTILS_ROBOT_UTILS_H
