#include "emergency_stopper/emergency_stopper.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "emergency_stopper");
  ros::NodeHandle nh;

  EmergencyStopper emergencyStopper(nh);

  try {
    emergencyStopper.spin();
  }
  catch (std::runtime_error& ex) {
    ROS_FATAL_STREAM("[emergency_stopper] Runtime error: " << ex.what());
    return 1;
  }

  return 0;
}
