#include "CsvScenarioHelper.h"
#include <fstream>
#include <sstream>

#define ROW_SIZE_TRAJECTORY 4
#define END_POINT_POSITIONS 3
#define BUILDING_DATA_COLUMNS 8

using namespace ns3::lrwpan;

NS_LOG_COMPONENT_DEFINE ( "CSV_HELPER" );

namespace ns3 {

void CsvScenarioHelper::LoadBuildings(const std::string& filename) {
    auto data = ReadCsv(filename);
    for (const auto& row : data) {
        if (row.size() < BUILDING_DATA_COLUMNS) continue;
        NS_LOG_INFO("Reading file: " << row[0] << ", " << row[1] << ", " << row[2] << ", " << row[3] << ", " << row[4] << ", " << row[5] << ", " << row[6] << ", " << row[7]);
        double xMin = std::stod(row[0]);
        double xMax = std::stod(row[1]);
        double yMin = std::stod(row[2]);
        double yMax = std::stod(row[3]);
        double zMin = std::stod(row[4]);
        double zMax = std::stod(row[5]);
        uint8_t buildingType = std::stod(row[6]);
        uint8_t wallType = std::stod(row[7]);
        Ptr<Building> b = CreateObject<Building> ();
        b->SetBoundaries(Box(xMin,xMax,yMin,yMax,zMin,zMax));
        b->SetBuildingType((ns3::Building::BuildingType_t)buildingType);
        b->SetExtWallsType((ns3::Building::ExtWallsType_t)wallType);

    }

}

void CsvScenarioHelper::LoadStaticPositions(const std::string& filename, NodeContainer& nodes) {
    auto data = ReadCsv(filename);
    uint32_t nodeIndex = 0;
    uint32_t totalNodes = nodes.GetN();
    // IMPORTANT: First position of the CSV is coordinator's position
    for (const auto& row : data) {
        if (row.size() < END_POINT_POSITIONS) continue;
        if (nodeIndex >= totalNodes - 1) {
            NS_LOG_WARN("CSV contains more positions than available static nodes. Ignoring extra rows.");
            break;
        }
        NS_LOG_INFO("Reading file: " << row[0] << ", " << row[1] << ", " << row[2]);
        double x = std::stod(row[0]);
        double y = std::stod(row[1]);
        double z = std::stod(row[2]);
         // Create and configure the ConstantPositionMobilityModel
        Ptr<ConstantPositionMobilityModel> staticMobility = CreateObject<ConstantPositionMobilityModel>();
        staticMobility->SetPosition(Vector(x, y, z));
        // Assign the mobility model directly to the node via aggregation
        nodes.Get(nodeIndex)->AggregateObject(staticMobility);
        nodeIndex++;
    }

}

double CsvScenarioHelper::LoadTrajectory(const std::string& filename, Ptr<WaypointMobilityModel> mobility, Ptr<Node> robot) {
    auto data = ReadCsv(filename);
    // Last instant of movement should cover most of SimulationTime
    double lastTrajectoryTime = 0;
    for (const auto& row : data) {
        if (row.size() < ROW_SIZE_TRAJECTORY) continue;
        NS_LOG_INFO("Reading file: " << row[0] << ", " << row[1] << ", " << row[2] << ", " << row[3]);
        double time = std::stod(row[0]);
        double x = std::stod(row[1]);
        double y = std::stod(row[2]);
        double z = std::stod(row[3]);
        mobility->AddWaypoint(Waypoint(Seconds(time), Vector(x, y, z)));
        lastTrajectoryTime = time;
    }

    // Finally, we add the mobility model directly to the Robot
    // IMPORTANT: While reading the way the physical layer sets its MobilityModel,
    // it's shown that you can either set it from that layer or, as last resort,
    // its DoInitialize method (executes at Simulator::Run) will get it from the node itself.
    // Some parts of the simulator probably depend on the same fallback.
    // So it's a better method to set it with node->AggregateObject(mobility)
    // instead of using the helper.
    robot->AggregateObject(mobility);
    return lastTrajectoryTime;
}

std::vector<std::vector<std::string>> CsvScenarioHelper::ReadCsv(const std::string& filename) {
    std::vector<std::vector<std::string>> results;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        NS_FATAL_ERROR("Could not open file: " << filename);
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue; // Skip comments

        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;
        while (std::getline(ss, value, ',')) {
            row.push_back(value);
        }
        results.push_back(row);
    }
    file.close();
    return results;
}

} // namespace ns3