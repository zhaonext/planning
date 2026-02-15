#pragma once
#include "Common/common.h"
#include "pugixml.hpp"
#include "road.h"

class ZebraCrossing {
public:
    ZebraCrossing(int id, double x_start, double y_start, double x_end, double y_end)
        : id(id), x_start(x_start), y_start(y_start), x_end(x_end), y_end(y_end) {}

    int getId() const { return id; }
    std::pair<double, double> getStartPoint() const { return {x_start, y_start}; }
    std::pair<double, double> getEndPoint() const { return {x_end, y_end}; }

private:
    int id;
    double x_start, y_start, x_end, y_end;
};

class TrafficLight {
public:
    TrafficLight(int id, double x, double y, double z, const std::string& orientation)
        : id(id), x(x), y(y), z(z), orientation(orientation), state(TrafficLightState::RED) {}

    int getId() const { return id; }
    std::tuple<double, double, double> getPosition() const { return std::make_tuple(x, y, z); }
    std::string getOrientation() const { return orientation; }

    // 设置交通灯状态
    void setState(TrafficLightState newState) {
        state = newState;
    }

    // 获取交通灯状态
    TrafficLightState getState() const {
        return state;
    }

private:
    int id;
    double x, y, z;
    std::string orientation;
    TrafficLightState state;  // 添加状态成员变量
};

class Connection {
public:
    Connection(std::string id, int fromRoadID, int toRoadID, int fromLaneID, int toLaneID)
        : id_(std::move(id)), fromRoadID_(fromRoadID), toRoadID_(toRoadID), fromLaneID_(fromLaneID), toLaneID_(toLaneID) {}

    std::string getId() const { return id_; }
    int getFromRoadId() const { return fromRoadID_; }
    int getToRoadId() const { return toRoadID_; }
    int getFromLaneId() const { return fromLaneID_; }
    int getToLaneId() const { return toLaneID_; }
    const std::vector<Lane>& getRoutingLane() const { return routing_lane_; }
    void setRoutingLane(const std::vector<Lane>& routingLane) { routing_lane_ = routingLane; }
    const Lane& getConnectionLane() const { return connection_lane_; }
    void setConnectionLane(const Lane& connectionLane) { connection_lane_ = connectionLane; }

private:
    std::string id_;
    int fromRoadID_;
    int toRoadID_;
    int fromLaneID_;
    int toLaneID_;
    Lane connection_lane_;
    std::vector<Lane> routing_lane_;
};

class RoadNetwork {
public:
    bool loadFromXML(const std::string& filename) {
        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_file(filename.c_str());

        if (!result) {
            AERROR << "XML [" << filename << "] parsed with errors: " << result.description();
            return false;
        }

        pugi::xml_node road_network = doc.child("RoadNetwork");

        // road
        for (pugi::xml_node road_node : road_network.children("Road")) {
            int road_id = road_node.attribute("id").as_int();
            std::unordered_map<int, std::vector<std::vector<double>>> baseControlPoints;

            for (pugi::xml_node lane_node : road_node.children("Lane")) {
                int land_id = lane_node.attribute("id").as_int();
                std::vector<std::vector<double>> controlPointsForLane;

                pugi::xml_node bezier_curve = lane_node.child("BezierCurve");
                for (pugi::xml_node control_point : bezier_curve.children("ControlPoint")) {
                    double x = control_point.attribute("x").as_double();
                    double y = control_point.attribute("y").as_double();
                    controlPointsForLane.push_back({x, y});
                }

                baseControlPoints[land_id] = controlPointsForLane;
            }

            // Create Road instance
            auto road = std::make_shared<Road>(road_id, baseControlPoints, offset, horizon);
            roads_.push_back(road);
        }

        // zebraCrossing
        for (pugi::xml_node crossing_node : road_network.child("ZebraCrossings").children("ZebraCrossing")) {
            int crossing_id = crossing_node.attribute("id").as_int();
            double x_start = crossing_node.attribute("x_start").as_double();
            double y_start = crossing_node.attribute("y_start").as_double();
            double x_end = crossing_node.attribute("x_end").as_double();
            double y_end = crossing_node.attribute("y_end").as_double();

            auto crossing = std::make_shared<ZebraCrossing>(crossing_id, x_start, y_start, x_end, y_end);
            crossings_.push_back(crossing);
        }
        // traffic lights
        for (pugi::xml_node light_node : road_network.child("TrafficLights").children("TrafficLight")) {
            int light_id = light_node.attribute("id").as_int();
            double x = light_node.attribute("x").as_double();
            double y = light_node.attribute("y").as_double();
            double z = light_node.attribute("z").as_double();
            std::string orientation = light_node.attribute("orientation").as_string();

            auto light = std::make_shared<TrafficLight>(light_id, x, y, z, orientation);
            traffic_lights_.push_back(light);
        }

        // connection
        for (pugi::xml_node connection_node : road_network.child("Connections").children("Connection")) {
            std::string connection_name = connection_node.attribute("name").as_string();
            pugi::xml_node fromNode = connection_node.child("From");
            int fromRoadID = fromNode.attribute("road").as_int();
            int fromLaneID = fromNode.attribute("lane").as_int();
            pugi::xml_node toNode = connection_node.child("To");
            int toRoadID = toNode.attribute("road").as_int();
            int toLaneID = toNode.attribute("lane").as_int();

            // 查找对应的 Road 实例并获取车道信息
            auto fromRoad = findRoadById(fromRoadID);
            auto toRoad = findRoadById(toRoadID);
            Lane fromLane = fromRoad->getLaneByID(fromLaneID);
            Lane toLane = toRoad->getLaneByID(toLaneID);

            auto connection = std::make_shared<Connection>(connection_name, fromRoadID, toRoadID, fromLaneID, toLaneID);

            // 解析控制点
            std::vector<std::vector<double>> controlPointsForLane;
            for (pugi::xml_node control_point_node : connection_node.child("ControlPoints").children("ControlPoint")) {
                double x = control_point_node.attribute("x").as_double();
                double y = control_point_node.attribute("y").as_double();
                controlPointsForLane.push_back({x, y});
            }

            // 根据控制点创建贝塞尔曲线
            auto refLine = std::make_shared<ReferenceBezierCurve<BEZIER_ORDER>>(controlPointsForLane);
            Lane connection_lane(0, refLine, offset, horizon);

            std::vector<Lane> routing_lane;
            routing_lane.emplace_back(fromLane);
            routing_lane.emplace_back(connection_lane);
            routing_lane.emplace_back(toLane);

            connection->setRoutingLane(routing_lane);
            connection->setConnectionLane(connection_lane);

            connections_[connection_name] = connection;
            //            connections_.emplace_back(connection);
        }

        return true;
    }

    const std::vector<std::shared_ptr<Road>>& getRoads() const { return roads_; }
    const std::vector<std::shared_ptr<ZebraCrossing>>& getZebraCrossings() const { return crossings_; }
    const std::vector<std::shared_ptr<TrafficLight>>& getTrafficLights() const { return traffic_lights_; }
    const unordered_map<std::string, std::shared_ptr<Connection>>& getConnections() const {return connections_;}

    static void calculateZebraCrossingParameters(const std::shared_ptr<ZebraCrossing>& crossing, double& length, double& yaw) {
        auto [x_start, y_start] = crossing->getStartPoint();
        auto [x_end, y_end] = crossing->getEndPoint();

        // 计算长度
        length = sqrt(pow(x_end - x_start, 2) + pow(y_end - y_start, 2));

        // 计算朝向（偏航角）
        yaw = atan2(y_end - y_start, x_end - x_start);
    }
    static TrafficLightOrientation stringToTrafficLightOrientation(const std::string& orientation) {
        if (orientation == "VERTICAL") {
            return TrafficLightOrientation::VERTICAL;
        } else if (orientation == "HORIZONTAL") {
            return TrafficLightOrientation::HORIZONTAL;
        } else {
            // 处理无效输入或提供默认值
            std::cerr << "Invalid orientation: " << orientation << std::endl;
            return TrafficLightOrientation::VERTICAL;  // 或其他默认值
        }
    }
    // 辅助函数：根据 road ID 查找 Road 实例
    std::shared_ptr<Road> findRoadById(int roadId) const {
        auto it = std::find_if(roads_.begin(), roads_.end(),
                               [roadId](const std::shared_ptr<Road>& road) {
                                   return road->getId() == roadId;
                               });
        return (it != roads_.end()) ? *it : nullptr;
    }

private:
    double offset = 1.875;
    int horizon = 51;

    std::vector<std::shared_ptr<Road>> roads_;
    std::vector<std::shared_ptr<ZebraCrossing>> crossings_;
    std::vector<std::shared_ptr<TrafficLight>> traffic_lights_;
    //    std::vector<std::shared_ptr<Connection>> connections_;
    std::unordered_map<std::string, std::shared_ptr<Connection>> connections_;
};
