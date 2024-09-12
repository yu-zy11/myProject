
// Modified from the Drake framework.
// Copyright 2012-2022 Robot Locomotion Group @ CSAIL. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.  Redistributions
// in binary form must reproduce the above copyright notice, this list of
// conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.  Neither the name of
// the Massachusetts Institute of Technology nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "math/roll_pitch_yaw.h"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include <fmt/format.h>

#include "math/rotation_matrix.h"

namespace core {
namespace math {

template <typename T>
RotationMatrix<T> RollPitchYaw<T>::ToRotationMatrix() const {
  return RotationMatrix<T>(*this);
}

// Uses a quaternion and its associated rotation matrix `R` to accurately
// and efficiently calculate the roll-pitch-yaw angles (SpaceXYZ Euler angles)
// that underlie `this` RollPitchYaw, even when the pitch angle p is very
// near a singularity (e.g., when p is within 1.0e-6 of π/2 or -π/2).
// `quaternion`: unit quaternion with elements `[e0, e1, e2, e3]`.
// `R`: The RotationMatrix corresponding to `quaternion`.
// Returns: [r, p, y] with `-π <= r <= π`, `-π/2 <= p <= π/2, `-π <= y <= π`.
// Note: The caller of this function is responsible for ensuring `quaternion`
// satisfies `e0^2 + e1^2 + e2^2 + e3^2 = 1` and that the matrix `R` is the
// rotation matrix that corresponds to `quaternion'.
//
// Theory
//
// This algorithm was created October 2016 by Paul Mitiguy for TRI (Toyota).
// We believe this is a new algorithm (not previously published).
// Some theory/formulation of this algorithm is provided below.  More detail
// is in Chapter 6 Rotation Matrices II [Mitiguy 2017] (reference below).
//
// Notation: Angles q1, q2, q3 designate SpaceXYZ "roll, pitch, yaw" angles.
//           A quaternion can be defined in terms of an angle-axis rotation by
//           an angle `theta` about a unit vector `lambda`.  For example,
//           consider right-handed orthogonal unit vectors Ax, Ay, Az and
//           Bx, By, Bz fixed in a frame A and a rigid body B, respectively.
//           Initially, Bx = Ax, By = Ay, Bz = Az, then B is subjected to a
//           right-handed rotation relative to frame A by an angle `theta`
//           about `lambda = L1*Ax + L2*Ay + L3*Az = L1*Bx + L2*By + L3*Bz`.
//           The elements of `quaternion` are defined e0, e1, e2, e3 as
//           `e0 = cos(theta/2)`, `e1 = L1*sin(theta/2)`,
//           `e2 = L2*sin(theta/2)`, `e3 = L3*sin(theta/2)`.
//
// Step 1. The 3x3 rotation matrix R is only used (in conjunction with the
//         atan2 function) to accurately calculate the pitch angle q2.
//         Note: Since only 5 elements of R are used, the algorithm could be
//         made slightly more efficient by computing/passing only those 5
//         elements (e.g., not calculating the other 4 elements) and/or
//         manipulating the relationship between `R` and quaternion to
//         further reduce calculations.
//
// Step 2. Realize the quaternion passed to the function can be regarded as
//         resulting from multiplication of certain 4x4 and 4x1 matrices, or
//         multiplying three rotation quaternions (Hamilton product), to give:
//         e0 = sin(q1/2)*sin(q2/2)*sin(q3/2) + cos(q1/2)*cos(q2/2)*cos(q3/2)
//         e1 = sin(q3/2)*cos(q1/2)*cos(q2/2) - sin(q1/2)*sin(q2/2)*cos(q3/2)
//         e2 = sin(q1/2)*sin(q3/2)*cos(q2/2) + sin(q2/2)*cos(q1/2)*cos(q3/2)
//         e3 = sin(q1/2)*cos(q2/2)*cos(q3/2) - sin(q2/2)*sin(q3/2)*cos(q1/2)
//
//         Reference for step 2: Chapter 6 Rotation Matrices II [Mitiguy 2017]
//
// Step 3.  Since q2 has already been calculated (in Step 1), substitute
//          cos(q2/2) = A and sin(q2/2) = f*A.
//          Note: The final results are independent of A and f = tan(q2/2).
//          Note: -pi/2 <= q2 <= pi/2  so -0.707 <= [A = cos(q2/2)] <= 0.707
//          and  -1 <= [f = tan(q2/2)] <= 1.
//
// Step 4.  Referring to Step 2 form: (1+f)*e1 + (1+f)*e3 and rearrange to:
//          sin(q1/2+q3/2) = (e1+e3)/(A*(1-f))
//
//          Referring to Step 2 form: (1+f)*e0 - (1+f)*e2 and rearrange to:
//          cos(q1/2+q3/2) = (e0-e2)/(A*(1-f))
//
//          Combine the two previous results to produce:
//          1/2*( q1 + q3 ) = atan2( e1+e3, e0-e2 )
//
// Step 5.  Referring to Step 2 form: (1-f)*e1 - (1-f)*e3 and rearrange to:
//          sin(q1/5-q3/5) = -(e1-e3)/(A*(1+f))
//
//          Referring to Step 2 form: (1-f)*e0 + (1-f)*e2 and rearrange to:
//          cos(q1/2-q3/2) = (e0+e2)/(A*(1+f))
//
//          Combine the two previous results to produce:
//          1/2*( q1 - q3 ) = atan2( e3-e1, e0+e2 )
//
// Step 6.  Combine Steps 4 and 5 and solve the linear equations for q1, q3.
//          Use zA, zB to handle case in which both atan2 arguments are 0.
//          zA = (e1+e3==0  &&  e0-e2==0) ? 0 : atan2( e1+e3, e0-e2 );
//          zB = (e3-e1==0  &&  e0+e2==0) ? 0 : atan2( e3-e1, e0+e2 );
//          Solve: 1/2*( q1 + q3 ) = zA     To produce:  q1 = zA + zB
//                 1/2*( q1 - q3 ) = zB                  q3 = zA - zB
//
// Step 7.  As necessary, modify angles by 2*PI to return angles in range:
//          -pi   <= q1 <= pi
//          -pi/2 <= q2 <= pi/2
//          -pi   <= q3 <= pi
//
// [Mitiguy, 2017]: "Advanced Dynamics and Motion Simulation,
//                   For professional engineers and scientists,"
//                   Prodigy Press, Sunnyvale CA, 2017 (Paul Mitiguy).
//                   Available at www.MotionGenesis.com
//
// Note: This algorithm is specific to SpaceXYZ (roll-pitch-yaw) order.
// It is easily modified for other SpaceIJK and BodyIJI rotation sequences.
// Author: Paul Mitiguy
template <typename T>
Eigen::Vector3<T> CalcRollPitchYawFromQuaternionAndRotationMatrix(const Eigen::Quaternion<T>& quaternion,
                                                                  const Eigen::Matrix3<T>& R) {
  // TODO: This method needs testing with symbolic template type T.
  //  Check if it works or throw a nice exception message.
  using std::abs;
  using std::atan2;
  using std::sqrt;

  // Calculate q2 using lots of information in the rotation matrix.
  // Rsum = abs( cos(q2) ) is inherently non-negative.
  // R20 = -sin(q2) may be negative, zero, or positive.
  const T R22 = R(2, 2);
  const T R21 = R(2, 1);
  const T R10 = R(1, 0);
  const T R00 = R(0, 0);
  const T Rsum = sqrt((R22 * R22 + R21 * R21 + R10 * R10 + R00 * R00) / 2);
  const T R20 = R(2, 0);
  const T q2 = atan2(-R20, Rsum);

  // Calculate q1 and q3 from Steps 2-6 (documented above).
  const T e0 = quaternion.w(), e1 = quaternion.x();
  const T e2 = quaternion.y(), e3 = quaternion.z();
  const T yA = e1 + e3, xA = e0 - e2;
  const T yB = e3 - e1, xB = e0 + e2;
  const T epsilon = Eigen::NumTraits<T>::epsilon();
  const auto isSingularA = abs(yA) <= epsilon && abs(xA) <= epsilon;
  const auto isSingularB = abs(yB) <= epsilon && abs(xB) <= epsilon;
  const T zA = (isSingularA ? T{0.0} : atan2(yA, xA));
  const T zB = (isSingularB ? T{0.0} : atan2(yB, xB));
  T q1 = zA - zB;  // First angle in rotation sequence.
  T q3 = zA + zB;  // Third angle in rotation sequence.

  // If necessary, modify angles q1 and/or q3 to be between -pi and pi.
  q1 = (q1 > M_PI ? q1 - 2 * M_PI : q1);
  q1 = (q1 < -M_PI ? q1 + 2 * M_PI : q1);
  q3 = (q3 > M_PI ? q3 - 2 * M_PI : q3);
  q3 = (q3 < -M_PI ? q3 + 2 * M_PI : q3);

  // Return in Drake/ROS conventional SpaceXYZ q1, q2, q3 (roll-pitch-yaw) order
  // (which is equivalent to BodyZYX q3, q2, q1 order).
  return Eigen::Vector3<T>(q1, q2, q3);
}

template <typename T>
void RollPitchYaw<T>::SetFromRotationMatrix(const RotationMatrix<T>& R) {
  SetFromQuaternionAndRotationMatrix(R.ToQuaternion(), R);
}

template <typename T>
void RollPitchYaw<T>::SetFromQuaternion(const Eigen::Quaternion<T>& quaternion) {
  SetFromQuaternionAndRotationMatrix(quaternion, RotationMatrix<T>(quaternion));
}

template <typename T>
void RollPitchYaw<T>::SetFromQuaternionAndRotationMatrix(const Eigen::Quaternion<T>& quaternion,
                                                         const RotationMatrix<T>& R) {
  const Eigen::Vector3<T> rpy = CalcRollPitchYawFromQuaternionAndRotationMatrix(quaternion, R.matrix());
  SetOrThrowIfNotValidInDebugBuild(rpy);

  // if constexpr (DCHECK_IS_ON()) {
  //   // Verify that arguments to this method make sense.  Ensure the
  //   // rotation_matrix and quaternion correspond to the same orientation.
  //   constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  //   const RotationMatrix<T> R_quaternion(quaternion);
  //   constexpr double tolerance = 20 * kEpsilon;
  //   if (!R_quaternion.IsNearlyEqualTo(R, tolerance)) {
  //     std::string message = fmt::format(
  //         "RollPitchYaw::{}():"
  //         " An element of the RotationMatrix R passed to this method differs by"
  //         " more than {:G} from the corresponding element of the RotationMatrix"
  //         " formed by the Quaternion passed to this method.  To avoid this"
  //         " inconsistency, ensure the orientation of R and Quaternion align."
  //         " ({}:{}).",
  //         __func__, tolerance, __FILE__, __LINE__);
  //     throw std::runtime_error(message);
  //   }

  //   // This algorithm converts a quaternion and RotationMatrix to RollPitchYaw.
  //   // It is tested by converting the returned RollPitchYaw to a RotationMatrix
  //   // and verifying the rotation matrices are within kEpsilon of each other.
  //   // Assuming sine, cosine are accurate to 4*(standard double-precision epsilon
  //   // = 2.22e-16) and there are two sets of two multiplies and one addition for
  //   // each rotation matrix element, I decided to test with 20 * kEpsilon:
  //   // (1+4*eps)*(1+4*eps)*(1+4*eps) = 1 + 3*(4*eps) + 3*(4*eps)^2 + (4*eps)^3.
  //   // Each + or * or sqrt rounds-off, which can introduce 1/2 eps for each.
  //   // Use: (12*eps) + (4 mults + 1 add) * 1/2 eps = 17.5 eps.
  //   const RollPitchYaw<T> roll_pitch_yaw(rpy);
  //   const RotationMatrix<T> R_rpy = RotationMatrix<T>(roll_pitch_yaw);
  //   DCHECK(R_rpy.IsNearlyEqualTo(R, 20 * kEpsilon));
  // }
}

template <typename T>
bool RollPitchYaw<T>::IsNearlySameOrientation(const RollPitchYaw<T>& other, double tolerance) const {
  // Note: When pitch is close to PI/2 or -PI/2, derivative calculations for
  // Euler angles can encounter numerical problems (dividing by nearly 0).
  // Although values of angles may "jump around" (difficult derivatives), the
  // angles' values should be able to be accurately reproduced.
  const RotationMatrix<T> R1(*this);
  const RotationMatrix<T> R2(other);
  return R1.IsNearlyEqualTo(R2, tolerance);
}

template <typename T>
void RollPitchYaw<T>::ThrowPitchAngleViolatesGimbalLockTolerance(const char* function_name, const char* file_name,
                                                                 int line_number, T pitch) {
  const double pitch_radians = pitch;
  const double cos_pitch = std::cos(pitch_radians);
  auto ret = DoesCosPitchAngleViolateGimbalLockTolerance(cos_pitch);
  if (!ret) {
    throw std::runtime_error("DoesCosPitchAngleViolateGimbalLockTolerance Error");
  }
  const double tolerance_degrees = GimbalLockPitchAngleTolerance() * 180 / M_PI;
  std::string message = fmt::format(
      "RollPitchYaw::{}():"
      " Pitch angle p = {:G} degrees is within {:G} degrees of gimbal-lock."
      " There is a divide-by-zero error (singularity) at gimbal-lock.  Pitch"
      " angles near gimbal-lock cause numerical inaccuracies.  To avoid this"
      " orientation singularity, use a quaternion -- not RollPitchYaw."
      " ({}:{}).",
      function_name, pitch_radians * 180 / M_PI, tolerance_degrees, file_name, line_number);
  throw std::runtime_error(message);
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const RollPitchYaw<T>& rpy) {
  T roll = rpy.roll();
  T pitch = rpy.pitch();
  T yaw = rpy.yaw();
  out << fmt::format("rpy = {} {} {}", roll, pitch, yaw);
  return out;
}

template std::ostream& operator<<<double>(std::ostream& out, const RollPitchYaw<double>& rpy);
template std::ostream& operator<<<float>(std::ostream& out, const RollPitchYaw<float>& rpy);

}  // namespace math
}  // namespace core

template class core::math::RollPitchYaw<double>;
template class core::math::RollPitchYaw<float>;