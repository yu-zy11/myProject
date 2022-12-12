#ifndef SWING_LEG_PLANNER_H
#define SWING_LEG_PLANNER_H
#include "./controller/common_data/common_data.h"
class SwingLegPlanner {
public:
  void init();
  void update();
  void getFootTarget();
  void getJointTarget();
  int a = 1;

private:
  int b = 1;
};
#endif
