// echo_brake.cpp

#include <ros/ros.h>
#include <std_msgs/Float32.h>

void brakeCallback(const std_msgs::Float32::ConstPtr &msg) {
    ROS_INFO("Brake: %.2f", msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "echo_brake");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/control/brake", 1, brakeCallback);

    ros::spin();
    return 0;
}
