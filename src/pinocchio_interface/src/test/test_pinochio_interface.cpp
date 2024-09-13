#include <gtest/gtest.h>

#include <iostream>
#include "pinocchio_interface/basic_structure.h"
#include "pinocchio_interface/pinocchio_interface.h"

TEST(TestPinocchioKinematics, testPinocchioInverseKinematics) {
  pino::PinocchioModelInfo model_info;
  {
    model_info.urdf_file_path = "/home/ubuntu/workspace/myProject/src/pinocchio_interface/src/test/SA01p.urdf";
    model_info.base_link_name = "base_link";
    model_info.end_effector_names = {"leg_l_foot_center", "leg_r_foot_center"};
    model_info.contact_type = {pino::ContactType::kSixDofContact, pino::ContactType::kSixDofContact};
    model_info.use_floating_base = true;
    model_info.print_pinocchio_info = false;
  };
  pino::PinocchioInterface pinocchio_interface(model_info);  // GetEndEffectorNumber()
  ASSERT_EQ(pinocchio_interface.GetEndEffectorNumber(), model_info.end_effector_names.size());
}
