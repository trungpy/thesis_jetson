// echo_throttle.cpp

#include <ros/ros.h>
#include <std_msgs/Float32.h>

void throttleCallback(const std_msgs::Float32::ConstPtr &msg) {
    ROS_INFO("Throttle: %.2f", msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "echo_throttle");
    ros::NodeHandle nh;

    ros::Subscriber sub =
        nh.subscribe("/control/throttle", 1, throttleCallback);

    ros::spin();
    return 0;
}
