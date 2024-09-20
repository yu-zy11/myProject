

#ifndef PINOCCHIO_INTERFACE_PINOCCHIO_CENTROIDAL_DYNAMICS_H_
#define PINOCCHIO_INTERFACE_PINOCCHIO_CENTROIDAL_DYNAMICS_H_

#include "pinocchio_interface/pinocchio_interface.h"

namespace pino {
class PinocchioCentroidalDynamics {
 public:
  PinocchioCentroidalDynamics(std::shared_ptr<PinocchioInterface> pino_ptr) : pino_ptr_(pino_ptr){};

  /** get centroidal momentum map Ag: H=Ag*dq
   * @param [in] qpos: joint position
   * @return centroidal momentum map Ag
   */
  Eigen::MatrixXd GetCentroidalMomentumMap(const Eigen::VectorXd& qpos);

  /** get time variation of centroidal momentum map
   * @param [in] qpos: joint position
   * @return time variation of centroidal momentum map dAg
   */
  Eigen::MatrixXd GetCentroidalMomentumMapTimeVariation(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);

  /** get the total Centroidal momenta of the system expressed around the center of mass.
   * @param [in] qpos: joint position
   * @param [in] qvel: joint velocity
   * @return Centroidal momenta
   */
  Eigen::MatrixXd GetCentroidalMomentum(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);
  /** get the total Centroidal momenta of the system expressed around the center of mass.
   * @param [in] qpos: joint position
   * @param [in] qvel: joint velocity
   * @param [out] hg: Centroidal momenta
   * @param [out] com: Center of mass
   * @param [out] vel_com: Velocity of center of mass
   */
  void GetCentroidalMomentum(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel, Eigen::MatrixXd& hg,
                             Eigen::Vector3d& com, Eigen::Vector3d& vel_com);

  /** get the Centroidal momemtum and its time derivatives,  expressed around the center of mass
   * @param [in] qpos: joint position
   * @param [in] qvel: joint velocity
   * @param [in] qacc: joint acceleration
   * @return Centroidal momentum  time derivatives
   */
  Eigen::MatrixXd GetCentroidalMomentumTimeVariation(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel,
                                                     const Eigen::VectorXd& qacc);

  /** get the Centroidal momemtum and its time derivatives,  expressed around the center of mass
   * @param [in] qpos: joint position
   * @param [in] qvel: joint velocity
   * @param [in] qacc: joint acceleration
   * @return Centroidal momentum  time derivatives
   */
  void GetCentroidalMomentumTimeVariation(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel,
                                          const Eigen::VectorXd& qacc, Eigen::MatrixXd& hg, Eigen::MatrixXd& dhg,
                                          Eigen::Vector3d& com, Eigen::Vector3d& vel_com);
  /** get center of mass of the robot
   * @return center of mass
   * @note call GetCentroidalMomentumTimeVariation or GetCentroidalMomentum first
   * */
  Eigen::Vector3d GetCenterOfMass() { return pino_ptr_->GetData()->com[0]; };

  /** get velocity of center of mass of the robot
   * @return velocity of center of mass
   *
   */
  Eigen::Vector3d GetVelocityAtCenterOfMass() { return pino_ptr_->GetData()->vcom[0]; };

  // Eigen::MatrixXd GetRobotInertialInWorldFrame() { return pino_ptr_->GetData()->oYcrb[0].angular(); };
  /**
   * @param [in] endeffector_id: endeffector id
   * @return position of endeffector relative to center of mass in world frame
   * @note call   pinocchio::forwardKinematics();
                  pinocchio::updateFramePlacements();first
           can use PinocchioInterface::updateKinematicsData()to run these functions
   */
  Eigen::Vector3d GetEndEffectorPositionToComInWorldFrame(size_t endeffector_id);
  /**
   * @param [in] endeffector_name: endeffector name
   * @return position of endeffector relative to center of mass in world frame
   * @note call   pinocchio::forwardKinematics();
                  pinocchio::updateFramePlacements();first
   */
  Eigen::Vector3d GetEndEffectorPositionToComInWorldFrame(std::string endeffector_name);

 private:
  std::shared_ptr<PinocchioInterface> pino_ptr_;
};
}  // namespace pino

#endif
