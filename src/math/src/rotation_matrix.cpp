
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
#include "math/rotation_matrix.h"

#include <string>

#include <fmt/format.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace core {
namespace math {

namespace {
// Returns true if all the elements of a quaternion are zero, otherwise false.
template <typename T>
bool IsQuaternionZero(const Eigen::Quaternion<T>& quaternion) {
  // Note: This special-purpose function avoids memory allocation on the heap
  // that sometimes occurs in quaternion.coeffs().isZero().  Alternatively, the
  // pseudo-code below uses DiscardGradient to reduce memory allocations.
  // return math::DiscardGradient(quaternion.coeffs()).isZero(0);
  return quaternion.w() == 0.0 && quaternion.x() == 0.0 && quaternion.y() == 0.0 && quaternion.z() == 0.0;
}

template <typename T>
void ThrowIfAllElementsInQuaternionAreZero(const Eigen::Quaternion<T>& quaternion, const char* function_name) {
  if (IsQuaternionZero(quaternion)) {
    std::string message = fmt::format(
        "{}():"
        " All the elements in a quaternion are zero.",
        function_name);
    throw std::logic_error(message);
  }
}

template <typename T>
void ThrowIfAnyElementInQuaternionIsInfinityOrNaN(const Eigen::Quaternion<T>& quaternion, const char* function_name) {
  if (!quaternion.coeffs().allFinite()) {
    std::string message = fmt::format(
        "{}():"
        " Quaternion contains an element that is infinity or NaN.",
        function_name);
    throw std::logic_error(message);
  }
}
}  // namespace

template <typename T>
RotationMatrix<T> RotationMatrix<T>::MakeFromOneUnitVector(const Eigen::Vector3<T>& u_A, int axis_index) {
  // In Debug builds, verify axis_index is 0 or 1 or 2 and u_A is unit length.
  // DCHECK(axis_index >= 0 && axis_index <= 2) << axis_index;
  if (axis_index < 0 || axis_index > 2) {
    throw std::logic_error("axis_index must be 0 or 1 or 2.");
  }
  // if constexpr (DCHECK_IS_ON()) {
  //   ThrowIfNotUnitLength(u_A, __func__);
  // }

  // This method forms a right-handed orthonormal basis with u_A and two
  // internally-constructed unit vectors v_A and w_A.
  // Herein u_A, v_A, w_A are abbreviated u, v, w, respectively.
  // Conceptually, v = a x u / |a x u| and w = u x v where unit vector a is
  // chosen so it is not parallel to u.  To speed this method, we judiciously
  // choose the unit vector a and simplify the algebra as described below.
  //
  // To form a unit vector v perpendicular to the given unit vector u, we use
  // the fact that a x u is guaranteed to be non-zero and perpendicular to u
  // when a is a unit vector and a ≠ ±u .  We choose vector a ≠ ±u by
  // identifying uₘᵢₙ, the element of u with the smallest absolute value.
  // If uₘᵢₙ = ux = u(0), set a = [1  0  0] so a x u = [0  -uz  uy].
  // If uₘᵢₙ = uy = u(1), set a = [0  1  0] so a x u = [uz  0  -ux].
  // If uₘᵢₙ = uz = u(2), set a = [0  0  1] so a x u = [-uy  ux  0].
  // Because we select uₘᵢₙ as the element with the *smallest* magnitude, and
  // since |u|² = ux² + uy² + uz² = 1, we are guaranteed that for the two
  // elements that are not uₘᵢₙ, at least one must be non-zero.
  //
  // We define v = a x u / |a x u|.  From our choice of the unit vector a:
  // If uₘᵢₙ is ux, |a x u|² = uz² + uy² = 1 - ux² = 1 - uₘᵢₙ²
  // If uₘᵢₙ is uy, |a x u|² = uz² + ux² = 1 - uy² = 1 - uₘᵢₙ²
  // If uₘᵢₙ is uz, |a x u|² = uy² + ux² = 1 - uz² = 1 - uₘᵢₙ²
  // The pattern shows that regardless of which element is uₘᵢₙ, in all cases,
  // |a x u| = √(1 - uₘᵢₙ²), hence v = a x u / √(1 - uₘᵢₙ²) which is written
  // as v = r a x u by defining r = 1 / √(1 - uₘᵢₙ²).  In view of a x u above:
  //
  // If uₘᵢₙ is ux, v = r a x u = [    0   -r uz    r uy].
  // If uₘᵢₙ is uy, v =           [ r uz       0   -r ux].
  // If uₘᵢₙ is uz, v =           [-r uy    r ux       0].
  //
  // By defining w = u x v, w is guaranteed to be a unit vector perpendicular
  // to both u and v and ordered so u, v, w form a right-handed set.
  // To avoid computational cost, we do not directly calculate w = u x v, but
  // instead substitute for v and do algebraic and vector simplification.
  //  w = u x v                  Next, substitute v = (a x u) / |a x u|.
  //    = u x (a x u) / |a x u|  Define r = 1 /|a x u| = 1 / √(1 - uₘᵢₙ²).
  //    = r u x (a x u)          Use vector triple product "bac-cab" property.
  //    = r [a(u⋅u) - u(u⋅a)]    Next, Substitute u⋅u = 1 and u⋅a = uₘᵢₙ.
  //    = r (a - uₘᵢₙ u)         This shows w is mostly in the direction of a.
  //
  // By examining the value of w = r(a - uₘᵢₙ u) for each possible value of
  // uₘᵢₙ, a pattern emerges. Below we substitute and simplify for the ux case
  // (analogous results for the uy and uz cases are shown further below):
  //
  // If uₘᵢₙ is ux, w = ([1  0  0] - uₘᵢₙ [uₘᵢₙ  uy  uz]) / √(1 - uₘᵢₙ²)
  //                  = [1 - uₘᵢₙ²   -uₘᵢₙ uy   -uₘᵢₙ uz] / √(1 - uₘᵢₙ²)
  //                  = [|a x u|   -r uₘᵢₙ uy   -r uₘᵢₙ uz]
  //                  = [|a x u|         s uy         s uz]
  //
  // where s = -r uₘᵢₙ.  The results for w for all three axes show a pattern:
  //
  // If uₘᵢₙ is ux, w = [|a x u|      s uy         s uz ]
  // If uₘᵢₙ is uy, w = [  s ux     |a x u|        s uz ]
  // If uₘᵢₙ is uz, w = [  s ux       s uy       |a x u|]
  //
  // The properties of the unit vectors v and w are summarized as follows.
  // Denoting i ∈ {0, 1, 2} as the index of u corresponding to uₘᵢₙ, then
  // v(i) = 0 and w(i) is the most positive element of w.  Moreover, since
  // w = r (a - uₘᵢₙ u), it follows that if uₘᵢₙ = 0, then w = r a is only in
  // the +a direction and since unit vector a has two zero elements, it is
  // clear uₘᵢₙ = 0 results in w having two zero elements and w(i) = 1.
  //
  // Instantiate the final rotation matrix and write directly to it instead of
  // creating temporary values and subsequently copying.
  RotationMatrix<T> R_AB(internal::DoNotInitializeMemberFields{});
  R_AB.R_AB_.col(axis_index) = u_A;

  // The auto keyword below improves efficiency by allowing an intermediate
  // eigen type to use a column as a temporary - avoids copy.
  auto v = R_AB.R_AB_.col((axis_index + 1) % 3);
  auto w = R_AB.R_AB_.col((axis_index + 2) % 3);

  // Indices i, j, k are in cyclical order: 0, 1, 2 or 1, 2, 0 or 2, 0, 1 and
  // are used to cleverly implement the previous patterns (above) for v and w.
  // The value of the index i is determined by identifying uₘᵢₙ = u_A(i),
  // the element of u_A with smallest absolute value.
  int i;
  u_A.cwiseAbs().minCoeff(&i);  // uₘᵢₙ = u_A(i).
  const int j = (i + 1) % 3;
  const int k = (j + 1) % 3;

  // uₘᵢₙ is the element of the unit vector u with smallest absolute value,
  // hence uₘᵢₙ² has the range 0 ≤ uₘᵢₙ² ≤ 1/3.  Thus for √(1 - uₘᵢₙ²), the
  // argument (1 - uₘᵢₙ²) has range 2/3 ≤ (1 - uₘᵢₙ²) ≤ 1.0.
  // Hence, √(1 - uₘᵢₙ²) should not encounter √(0) or √(negative_number).
  // Since √(0) cannot occur, the calculation √(1 - uₘᵢₙ²) is safe for type
  // T = AutoDiffXd because we avoid NaN in derivative calculations.
  // Reminder: ∂(√(x)/∂x = 0.5/√(x) will not have a NaN if x > 0.
  using std::sqrt;
  const T mag_a_x_u = sqrt(1 - u_A(i) * u_A(i));  // |a x u| = √(1 - uₘᵢₙ²)
  const T r = 1 / mag_a_x_u;
  const T s = -r * u_A(i);
  v(i) = 0;
  v(j) = -r * u_A(k);
  v(k) = r * u_A(j);
  w(i) = mag_a_x_u;   // w(i) is the most positive component of w.
  w(j) = s * u_A(j);  // w(j) = w(k) = 0 if uₘᵢₙ (that is, u_A(i)) is zero.
  w(k) = s * u_A(k);

  return R_AB;
}

template <typename T>
RotationMatrix<T> RotationMatrix<T>::MakeFromOneUnitVectorAsZAxis(const Eigen::Vector3<T>& u_A) {
  // if constexpr (DCHECK_IS_ON()) {
  //   ThrowIfNotUnitLength(u_A, __func__);
  // }

  // This method(Duff, 2017) forms a right-handed orthonormal basis with a unit vector u_A as z axis and two
  // internally-constructed unit vectors v_A and w_A. This method is faster than function 'MakeFromOneUnitVector', since
  // it does not require the reciprocal square root computation and avoids a precision problem in Frisvad(2012).
  // Reference:
  // [1] Frisvad J R. Building an orthonormal basis from a 3D unit vector without normalization[J]. Journal of Graphics
  // Tools, 2012, 16(3): 151-159.
  // [2] Duff T, Burgess J, Christensen P, et al. Building an orthonormal basis, revisited[J]. JCGT, 2017, 6(1).
  //
  // Instantiates the final rotation matrix and write directly to it instead of creating temporary values and
  // subsequently copying.
  RotationMatrix<T> R_AB(internal::DoNotInitializeMemberFields{});
  R_AB.R_AB_.col(2) = u_A;

  // The auto keyword below improves efficiency by allowing an intermediate eigen type to use a column as a temporary -
  // avoids copy.
  auto v = R_AB.R_AB_.col(0);
  auto w = R_AB.R_AB_.col(1);

  T x = u_A.x();
  T y = u_A.y();
  T z = u_A.z();
  T sign = std::copysign(T{1}, z);
  const T a = -1 / (sign + z);
  const T b = x * y * a;
  v = Eigen::Vector3<T>{1 + sign * x * x * a, sign * b, -sign * x};
  w = Eigen::Vector3<T>{b, sign + y * y * a, -y};

  return R_AB;
}

template <typename T>
Eigen::Matrix3<T> RotationMatrix<T>::QuaternionToRotationMatrix(const Eigen::Quaternion<T>& quaternion,
                                                                T two_over_norm_squared) {
  ThrowIfAllElementsInQuaternionAreZero(quaternion, __func__);
  // if constexpr (DCHECK_IS_ON()) {
  //   ThrowIfAnyElementInQuaternionIsInfinityOrNaN(quaternion, __func__);
  // }

  T w = quaternion.w();
  T x = quaternion.x();
  T y = quaternion.y();
  T z = quaternion.z();
  const T sx = two_over_norm_squared * x;  // scaled x-value.
  const T sy = two_over_norm_squared * y;  // scaled y-value.
  const T sz = two_over_norm_squared * z;  // scaled z-value.
  const T swx = sx * w;
  const T swy = sy * w;
  const T swz = sz * w;
  const T sxx = sx * x;
  const T sxy = sy * x;
  const T sxz = sz * x;
  const T syy = sy * y;
  const T syz = sz * y;
  const T szz = sz * z;

  Eigen::Matrix3<T> m;
  m.coeffRef(0, 0) = T(1) - syy - szz;
  m.coeffRef(0, 1) = sxy - swz;
  m.coeffRef(0, 2) = sxz + swy;
  m.coeffRef(1, 0) = sxy + swz;
  m.coeffRef(1, 1) = T(1) - sxx - szz;
  m.coeffRef(1, 2) = syz - swx;
  m.coeffRef(2, 0) = sxz - swy;
  m.coeffRef(2, 1) = syz + swx;
  m.coeffRef(2, 2) = T(1) - sxx - syy;

  return m;
}

template <typename T>
void RotationMatrix<T>::ThrowIfNotValid(const Eigen::Matrix3<T>& R) {
  if (!R.allFinite()) {
    throw std::logic_error(
        "Error: Rotation matrix contains an element that is infinity or"
        " NaN.");
  }
  // If the matrix is not-orthogonal, try to give a detailed message.
  // This is particularly important if matrix is very-near orthogonal.
  if (!IsOrthonormal(R, get_internal_tolerance_for_orthonormality())) {
    const T measure_of_orthonormality = GetMeasureOfOrthonormality(R);
    const double measure = measure_of_orthonormality;
    std::string message = fmt::format(
        "Error: Rotation matrix is not orthonormal.\n"
        "  Measure of orthonormality error: {}  (near-zero is good).\n"
        "  To calculate the proper orthonormal rotation matrix closest to"
        " the alleged rotation matrix, use the SVD (expensive) static method"
        " RotationMatrix<T>::ProjectToRotationMatrix(), or for a less"
        " expensive (but not necessarily closest) rotation matrix, use"
        " RotationMatrix<T>(RotationMatrix<T>::ToQuaternion<T>(your_matrix))."
        " Alternatively, if using quaternions, ensure the quaternion is"
        " normalized.",
        measure);
    throw std::logic_error(message);
  }
  if (R.determinant() < 0) {
    throw std::logic_error(
        "Error: Rotation matrix determinant is negative."
        " It is possible a basis is left-handed.");
  }
}

template <typename T>
Eigen::Matrix3<T> RotationMatrix<T>::ProjectMatrix3ToOrthonormalMatrix3(const Eigen::Matrix3<T>& M, T* quality_factor) {
  const auto svd = M.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (quality_factor != nullptr) {
    // Singular values are always non-negative and sorted in decreasing order.
    const auto singular_values = svd.singularValues();
    const T s_max = singular_values(0);  // maximum singular value.
    const T s_min = singular_values(2);  // minimum singular value.
    const T s_f = (s_max != 0.0 && s_min < 1.0 / s_max) ? s_min : s_max;
    const T det = M.determinant();
    const double sign_det = (det > 0.0) ? 1 : ((det < 0.0) ? -1 : 0);
    *quality_factor = s_f * sign_det;
  }
  return svd.matrixU() * svd.matrixV().transpose();
}

double ProjectMatToRotMatWithAxis(const Eigen::Matrix3d& M, const Eigen::Vector3d& axis, const double angle_lb,
                                  const double angle_ub) {
  if (angle_ub < angle_lb) {
    throw std::runtime_error(
        "The angle upper bound should be no smaller than the angle lower "
        "bound.");
  }
  const double axis_norm = axis.norm();
  if (axis_norm == 0) {
    throw std::runtime_error("The axis argument cannot be the zero vector.");
  }
  const Eigen::Vector3d a = axis / axis_norm;
  Eigen::Matrix3d A = Eigen::Matrix3d{
      {0, -a(2), a(1)},
      {a(2), 0, -a(0)},
      {-a(1), a(0), 0},
  };
  const double alpha = atan2(-(M.transpose() * A * A).trace(), (A.transpose() * M).trace());
  double theta{};
  // The bounds on θ + α is [angle_lb + α, angle_ub + α].
  if (std::isinf(angle_lb) && std::isinf(angle_ub)) {
    theta = M_PI_2 - alpha;
  } else if (std::isinf(angle_ub)) {
    // First if the angle upper bound is inf, start from the angle_lb, and
    // find the angle θ, such that θ + α = 0.5π + 2kπ
    const int k = ceil((angle_lb + alpha - M_PI_2) / (2 * M_PI));
    theta = (2 * k + 0.5) * M_PI - alpha;
  } else if (std::isinf(angle_lb)) {
    // If the angle lower bound is inf, start from the angle_ub, and find the
    // angle θ, such that θ + α = 0.5π + 2kπ
    const int k = floor((angle_ub + alpha - M_PI_2) / (2 * M_PI));
    theta = (2 * k + 0.5) * M_PI - alpha;
  } else {
    // Now neither angle_lb nor angle_ub is inf. Check if there exists an
    // integer k, such that 0.5π + 2kπ ∈ [angle_lb + α, angle_ub + α]
    const int k = floor((angle_ub + alpha - M_PI_2) / (2 * M_PI));
    const double max_sin_angle = M_PI_2 + 2 * k * M_PI;
    if (max_sin_angle >= angle_lb + alpha) {
      // 0.5π + 2kπ ∈ [angle_lb + α, angle_ub + α]
      theta = max_sin_angle - alpha;
    } else {
      // Now the maximal is at the boundary, either θ = angle_lb or angle_ub
      if (sin(angle_lb + alpha) >= sin(angle_ub + alpha)) {
        theta = angle_lb;
      } else {
        theta = angle_ub;
      }
    }
  }
  return theta;
}

template <typename T>
void RotationMatrix<T>::ThrowIfNotUnitLength(const Eigen::Vector3<T>& v, const char* function_name) {
  // The value of kTolerance was determined empirically, is well within the
  // tolerance achieved by normalizing a vast range of non-zero vectors, and
  // seems to guarantee a valid RotationMatrix() (see IsValid()).
  constexpr double kTolerance = 4 * std::numeric_limits<double>::epsilon();
  const double norm = v.norm();
  const double error = std::abs(1.0 - norm);
  // Throw an exception if error is non-finite (NaN or infinity) or too big.
  if (!std::isfinite(error) || error > kTolerance) {
    const double vx = v.x();
    const double vy = v.y();
    const double vz = v.z();
    throw std::logic_error(
        fmt::format("RotationMatrix::{}() requires a unit-length vector.\n"
                    "         v: {} {} {}\n"
                    "       |v|: {}\n"
                    " |1 - |v||: {} is not less than or equal to {}.",
                    function_name, vx, vy, vz, norm, error, kTolerance));
  }
}

template <typename T>
Eigen::Vector3<T> RotationMatrix<T>::NormalizeOrThrow(const Eigen::Vector3<T>& v, const char* function_name) {
  Eigen::Vector3<T> u;
  // The number 1.0e-10 is a heuristic (rule of thumb) that is guided by
  // an expected small physical dimensions in a robotic systems.  Numbers
  // smaller than this are probably user or developer errors.
  constexpr double kMinMagnitude = 1.0e-10;
  const double norm = v.norm();
  // Normalize the vector v if norm is finite and sufficiently large.
  // Throw an exception if norm is non-finite (NaN or infinity) or too small.
  if (std::isfinite(norm) && norm >= kMinMagnitude) {
    u = v / norm;
  } else {
    const double vx = v.x();
    const double vy = v.y();
    const double vz = v.z();
    throw std::logic_error(
        fmt::format("RotationMatrix::{}() cannot normalize the given vector.\n"
                    "   v: {} {} {}\n"
                    " |v|: {}\n"
                    " The measures must be finite and the vector must have a"
                    " magnitude of at least {} to automatically normalize. If"
                    " you are confident that v's direction is meaningful, pass"
                    " v.normalized() in place of v.",
                    function_name, vx, vy, vz, norm, kMinMagnitude));
  }
  return u;
}

}  // namespace math
}  // namespace core

template class core::math::RotationMatrix<double>;
template class core::math::RotationMatrix<float>;