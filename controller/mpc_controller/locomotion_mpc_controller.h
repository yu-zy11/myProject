#ifndef PX_QUADRUPED_MODULES_LOCOMOTION_LOCOMOTION_CORE_CONTROLLER_MPC_CONTROLLER_LOCOMOTION_MPC_CONTROLLER_H_
#define PX_QUADRUPED_MODULES_LOCOMOTION_LOCOMOTION_CORE_CONTROLLER_MPC_CONTROLLER_LOCOMOTION_MPC_CONTROLLER_H_

// #include "modules/locomotion/locomotion_core/controller/based_controller.h"
// #include "modules/locomotion/locomotion_core/controller/mpc_controller/stance_leg_mpc_controller.h"
// #include "modules/locomotion/locomotion_core/gait/event_gait_generator.h"
// #include "modules/locomotion/locomotion_core/gait/time_gait_generator.h"
// #include "modules/locomotion/locomotion_core/planner/footstep_planner.h"
// #include "modules/locomotion/locomotion_core/planner/swing_trajectory_planner.h"
// #include "modules/locomotion/locomotion_core/utils/noise_reduction.h"
// #include "modules/locomotion/locomotion_core/utils/swing_leg_controller.h"

namespace MCC {
namespace locomotion_core {
class LocomotionMPCController : public BasedController {
 public:
  explicit LocomotionMPCController(MCC::common::robot::RobotStates state);

  LocomotionMPCController(const LocomotionMPCController&) = delete;

  LocomotionMPCController& operator=(const LocomotionMPCController&) = delete;

  virtual ~LocomotionMPCController();

  void Init(const common::message::FusionData& fusion_data);

  common::message::LocomotionData Run(
      const common::message::FusionData& fusion_data, const common::message::OperatorData& operator_data);

  void SetGaitType(GaitType type);

  MCC::common::message::GaitInfo UpdateGaitInfo() override;

  bool Safe() override;

 private:
  GaitGenerator* gait_generator_;
  FootstepPlanner* footstep_planner_;
  SwingTrajectoryPlanner* swing_trajectory_planner_;
  LegController* swing_leg_controller_;
  LegController* stance_leg_mpc_controller_;
  NosieReduction* nosiereduction_;
};
}  // namespace locomotion_core
}  // namespace MCC
#endif  // PX_QUADRUPED_MODULES_LOCOMOTION_LOCOMOTION_CORE_CONTROLLER_MPC_CONTROLLER_LOCOMOTION_MPC_CONTROLLER_H_
