#ifndef CSV_SCENARIO_HELPER_H
#define CSV_SCENARIO_HELPER_H

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/buildings-module.h"
#include "ns3/waypoint-mobility-model.h"
#include "ns3/network-module.h"
#include "ns3/lr-wpan-module.h"
#include <string>
#include <vector>

namespace ns3 {

class CsvScenarioHelper {
public:
    static void LoadBuildings(const std::string& filename);
    static void LoadStaticPositions(const std::string& filename, NodeContainer& nodes);
    static double LoadTrajectory(const std::string& filename, Ptr<WaypointMobilityModel> mobility, Ptr<Node> robot);

private:
    static std::vector<std::vector<std::string>> ReadCsv(const std::string& filename);
};

}

#endif 