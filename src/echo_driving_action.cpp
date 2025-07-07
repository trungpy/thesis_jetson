// echo_driving_action.cpp

#include <ros/ros.h>
#include <std_msgs/String.h>

void actionCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("Driving Action: %s", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "echo_driving_action");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/driving_action", 1, actionCallback);

    ros::spin();
    return 0;
}
