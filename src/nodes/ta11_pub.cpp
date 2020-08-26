#include "ros/ros.h"
#include "tiago_tactile_ta11/ta11.h"

int main (int argc, char** argv) {
  ros::init(argc, argv, "ta11_pub");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  tiago_tactile_ta11::TA11 t;
  t.read_loop();

  return 0;
}