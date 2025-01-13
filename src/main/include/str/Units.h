#pragma once

#include <units/base.h>
#include <units/math.h>

namespace units {

template <typename T>
bool essentiallyEqual(T a, T b, units::scalar_t epsilon) {
  return units::math::abs(a - b) <=
         ((units::math::abs(a) > units::math::abs(b) ? units::math::abs(b)
                                                     : units::math::abs(a)) *
          epsilon);
}
}  // namespace units
