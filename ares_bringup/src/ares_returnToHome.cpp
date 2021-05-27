#include <ros/ros.h>
// #include <sensor_msgs/BatteryState.h>
// #include <std_msgs/Bool.h>
// #include <actionlib/client/simple_action_client.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <move_base_msgs/MoveBaseAction.h>

// actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

// void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
// {
//   if(msg.voltage < 11.3)
//   {
//     move_base_msgs::MoveBaseGoal goal;
//     move_base_client_.cancelAllGoals();
//     move_base_client_.sendGoal(goal)

//   }
// }

int main(int argc, char **argv)
{


  ros::init(argc, argv, "listener");


  // ros::NodeHandle nh;

  // ROS_INFO("Waiting to connect to move_base server");
  // move_base_client_.waitForServer();
  // ROS_INFO("Connected to move_base server");


  // ros::Subscriber sub = nh.subscribe("/battery_state", 10, batteryCallback);


  // ros::spin();

  return 0;
}