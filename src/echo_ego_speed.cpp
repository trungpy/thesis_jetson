// echo_ego_speed.cpp
#include <ros/ros.h>
#include <std_msgs/Float32.h>
void speedCallback(const std_msgs::Float32::ConstPtr &msg) {
    ROS_INFO("Ego Speed: %.2f", msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "echo_ego_speed");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/ego_speed", 1, speedCallback);

    ros::spin();
    return 0;
}