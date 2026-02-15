#pragma once
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "traffic_light_controller.h"

class TrafficLightControllerNode {
public:
    TrafficLightControllerNode() : controller(), nh("~"), rate(1) {
        light_pub = nh.advertise<std_msgs::String>("traffic_light_status", 10);
    }

    void run() {
        while (ros::ok()) {
            controller.update(1);  // 假设每次循环1秒
            publishStatus();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    TrafficLightController controller;
    ros::NodeHandle nh;
    ros::Publisher light_pub;
    ros::Rate rate;

    void publishStatus() {
        for (int i = 1; i <= 4; ++i) {
            std_msgs::String msg;
            std::stringstream ss;
            ss << "Light " << i << ": " << controller.getStateStringForLight(i);

            msg.data = ss.str();
            light_pub.publish(msg);
        }
    }
};
