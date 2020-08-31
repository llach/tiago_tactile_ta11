#include "ros/ros.h"
#include <thread>
#include "tiago_tactile_ta11/ta11.h"
#include "tiago_tactile_msgs/TA11.h"

int main (int argc, char** argv) {
  ros::init(argc, argv, "ta11_pub");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<tiago_tactile_msgs::TA11>("/ta11", 1);
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::vector<std::string> sensor_frames = {"ta11_left_finger_link", "ta11_right_finger_link"};

  tiago_tactile_ta11::TA11 t;
  std::thread th(&tiago_tactile_ta11::TA11::read_loop, std::ref(t));

  ros::Rate r(50);

  while (ros::ok()){
    tiago_tactile_msgs::TA11 tac;
    tac.header.frame_id = "base_link";

    tac.frame_names.push_back(sensor_frames[0]);
    tac.frame_names.push_back(sensor_frames[1]);

    {
      const std::lock_guard<std::mutex> lock(t.values_lock);
      tac.sensor_values.push_back(t.values[0]);
      tac.sensor_values.push_back(t.values[1]);
    }

    pub.publish(tac);
    r.sleep();
  }

  th.join();
  return 0;
}