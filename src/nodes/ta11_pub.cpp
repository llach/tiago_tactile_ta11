#include "ros/ros.h"
#include <thread>
#include "tiago_tactile_ta11/ta11.h"
#include "std_msgs/Float64MultiArray.h"

int main (int argc, char** argv) {
  ros::init(argc, argv, "ta11_pub");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/ta11", 1);
  ros::AsyncSpinner spinner(4);
  spinner.start();

  tiago_tactile_ta11::TA11 t;
  std::thread th(&tiago_tactile_ta11::TA11::read_loop, std::ref(t));

  ros::Rate r(50);

  while (ros::ok()){
    std_msgs::Float64MultiArray fla;

    {
      const std::lock_guard<std::mutex> lock(t.values_lock);
      fla.data.push_back(t.values[0]);
      fla.data.push_back(t.values[1]);
    }

    pub.publish(fla);
    r.sleep();
  }

  th.join();
  return 0;
}