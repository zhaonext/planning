#include "Scenarios/scenarios.h"

int main() {
    std::vector<ObstacleInfo> obstacles;
    ScenarioManager::loadParam("/home/vtd/Documents/routing_planning/code/iLQR/TrajOpt/Params/scenarios.json", "tailgating", obstacles);

    ScenarioManager::print_weight_bound();

    for (const auto& obstacle : obstacles) {
        obstacle.print_info();
    }
    return 0;
}
