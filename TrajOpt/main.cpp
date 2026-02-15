#include <ostream>
#include <streambuf>

#include "Interfaces/planner_node.h"
// #include "SamplingTrajectory/sample_trajectory.h"
// #include "TrafficRules/traffic_light_node.h"
namespace plt = matplotlibcpp;

class NullStreambuf : public std::streambuf {
public:
    int overflow(int c) override {
        return c;
    }
};

NullStreambuf null_buf;
std::ostream dev_null(&null_buf);

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    double sim_step = 0.1;
    ros::Rate rate(1.0 / sim_step);
    PlannerNode planner_node(nh);

    while (ros::ok()) {
        planner_node.planCallback();
        ros::spinOnce();
        rate.sleep();
    }
}
