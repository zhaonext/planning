#pragma once

class TrafficLightController {
public:
    TrafficLightController()
        : elapsedTime(0), phaseDuration({500, 0, 00}) {}

    void update(int deltaTime) {
        elapsedTime += deltaTime;
        int totalCycleDuration = std::accumulate(phaseDuration.begin(), phaseDuration.end(), 0);
        int timeInCycle = elapsedTime % totalCycleDuration;
        int accumulatedTime = 0;

        for (int i = 0; i < phaseDuration.size(); ++i) {
            accumulatedTime += phaseDuration[i];
            if (timeInCycle < accumulatedTime) {
                setCurrentPhase(i);
                break;
            }
        }
    }

    TrafficLightState getStateForLight(int lightId) const {
        if (lights.find(lightId) != lights.end()) {
            return lights.at(lightId);
        }
        return TrafficLightState::RED;
    }

    std::string getStateStringForLight(int lightId) const {
        TrafficLightState state = getStateForLight(lightId);
        switch (state) {
            case TrafficLightState::RED:
                return "RED";
            case TrafficLightState::YELLOW:
                return "YELLOW";
            case TrafficLightState::GREEN:
                return "GREEN";
            // 可能还有其他状态
            default:
                return "UNKNOWN";
        }
    }

private:
    std::map<int, TrafficLightState> lights;
    int elapsedTime;
    std::vector<int> phaseDuration;  // 持续时间序列：东西绿，东西黄，南北绿，南北黄

    void setCurrentPhase(int phase) {
        switch (phase) {
            case 0:  // 绿灯
                lights[3] = TrafficLightState::GREEN;
                break;
            case 1:  // 黄灯
                lights[3] = TrafficLightState::YELLOW;
                break;
            case 2:  // 红灯
                lights[3] = TrafficLightState::RED;
                break;
            default:
                lights[3] = TrafficLightState::RED;
        }
    }

    //    void setCurrentPhase(int phase) {
    //        switch (phase) {
    //            case 0:  // 东西绿，南北红
    //                lights[1] = lights[3] = TrafficLightState::GREEN;
    //                lights[2] = lights[4] = TrafficLightState::RED;
    //                break;
    //            case 1:  // 东西黄，南北红
    //                lights[1] = lights[3] = TrafficLightState::YELLOW;
    //                lights[2] = lights[4] = TrafficLightState::RED;
    //                break;
    //            case 2:  // 东西红，南北绿
    //                lights[1] = lights[3] = TrafficLightState::RED;
    //                lights[2] = lights[4] = TrafficLightState::GREEN;
    //                break;
    //            case 3:  // 东西红，南北黄
    //                lights[1] = lights[3] = TrafficLightState::RED;
    //                lights[2] = lights[4] = TrafficLightState::YELLOW;
    //                break;
    //        }
    //    }
};
