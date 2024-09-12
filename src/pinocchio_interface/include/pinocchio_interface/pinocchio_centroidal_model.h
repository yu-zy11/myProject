

#ifndef PINOCCHIO_INTERFACE_PINOCCHIO_CENTROIDAL_DYNAMICS_H_
#define PINOCCHIO_INTERFACE_PINOCCHIO_CENTROIDAL_DYNAMICS_H_

// clang-format off
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <Eigen/Dense>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal-derivatives.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <string>
#include <vector>
// clang-format on

namespace pino {
class PinocchioCentroidalModel {
  using Model = pinocchio::ModelTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;
  using Data = pinocchio::DataTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;

 public:
  PinocchioCentroidalModel(std::shared<Model> model_ptr, std::shared<Data> data_ptr,
                           std::shared_ptr<PinocchioModelInfo> model_info)
      : model_ptr_(model_ptr), data_ptr_(data_ptr), info_ptr_(model_info){};
  ~PinocchioCentroidalModel();

  std::shared_ptr<Model> model_ptr_;
  std::shared_ptr<Data> data_ptr_;
  std::shared_ptr<PinocchioModelInfo> info_ptr_;
};
}  // namespace pino

#endif
