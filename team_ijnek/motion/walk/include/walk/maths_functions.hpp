// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <cmath>

#ifndef WALK__MATHS_FUNCTIONS_HPP_
#define WALK__MATHS_FUNCTIONS_HPP_

float parabolicStep(float dt, float time, float period, float deadTimeFraction = 0);
float parabolicReturnMod(float f);
float linearStep(float time, float period);

template <typename T>
inline static T crop(const T &x, const T &minimum, const T &maximum)
{
   if (x < minimum) {
      return minimum;
   } else if (x > maximum) {
      return maximum;
   } else {
      return x;
   }
}


#endif  // WALK__MATHS_FUNCTIONS_HPP_
