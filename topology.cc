//
// Topología de Red WSN sobre 802.15.4  
// para cálculo de SLAM
//
//
// Integrantes: Juan del Pozo Ávila, Ulfer Cit Flores Taco, Sébastien Deurveilher
// Curso 2025-2026 
//

// INCLUDES

// FROM NS3

#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/packet.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/single-model-spectrum-channel.h"
#include "ns3/waypoint-mobility-model.h"
#include "ns3/waypoint.h"

// FROM C++

#include <vector>
#include <fstream>
#include <iostream>

#include "entities.h"
#include "SlamDataCollector.h"

// DEFINES

#define PAN_ID 0xAAAA
#define NUMBER_COORDINATORS 1
#define NUMBER_EDS 8
#define NUMBER_ROBOTS 1
#define BROADCAST_ADDR "ff:ff"

using namespace ns3;
using namespace ns3::lrwpan;

NS_LOG_COMPONENT_DEFINE ( "RSSI-TOPOLOGY" );

int
main(int argc, char* argv[])
{

    Time::SetResolution(Time::PS);
    std::string trajectoryFilename = "trajectory.csv";
    std::string nodePositions = "positions.csv";
    bool pcapTracing = false;
    bool verificationMode = false;

    CommandLine parameters;
    parameters.AddValue("pcap", "Flag for generating pcap of traffic",pcapTracing);
    parameters.AddValue("trajectoryFilename","Filename of Mobility Data",trajectoryFilename);
    parameters.AddValue("nodePositions","Filename of Mobility Data",nodePositions);
    parameters.AddValue("verif", "Flag for running the simulation in verification mode" , verificationMode);
    parameters.Parse(argc,argv);

    // Helper Configuration
    LrWpanHelper lrWpanHelper;
    lrWpanHelper.SetPropagationDelayModel("ns3::ConstantSpeedPropagationDelayModel");
    lrWpanHelper.AddPropagationLossModel("ns3::LogDistancePropagationLossModel"); // NakagamiPropagationLossModel could be implemented in the future
    NodeContainer nodes;
    nodes.Create(NUMBER_COORDINATORS+NUMBER_EDS+NUMBER_ROBOTS); 
    NetDeviceContainer devices = lrWpanHelper.Install(nodes);

    // After Installing
    // =========================================================================
    // 1. PAN Creation
    // =========================================================================
    lrWpanHelper.CreateAssociatedPan(devices, PAN_ID);

    // =========================================================================
    // 2. MobilityModel configuration for the Robot (Dynamic)
    // =========================================================================
    Ptr<Node> robot = nodes.Get(nodes.GetN() - 1);
    Ptr<WaypointMobilityModel> robotMobilityModel = CreateObject<WaypointMobilityModel>();

    // Open and Read trajectoryFilename
    std::ifstream trajectoryFile(trajectoryFilename);
    if (!trajectoryFile.is_open()) {
        NS_FATAL_ERROR("Error opening: " << trajectoryFilename);
    }

    // Last instant of movement should be SimulationTime
    double tSim;

    std::string line;
    while (std::getline(trajectoryFile, line)) {
        // Ignoring comments or empty lines
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        std::string item;
        double t, x, y, z;

        // Parse the 4 comma-separated values
        if (std::getline(ss, item, ',')) t = std::stod(item);
        if (std::getline(ss, item, ',')) x = std::stod(item);
        if (std::getline(ss, item, ',')) y = std::stod(item);
        if (std::getline(ss, item, ',')) z = std::stod(item);

        robotMobilityModel->AddWaypoint(Waypoint(Seconds(t), Vector(x, y, z)));
        tSim = t;

    }
    trajectoryFile.close();

    // Finally, we add the mobility model directly to the node
    // IMPORTANT: While reading the way the physical layer sets its MobilityModel,
    // it's shown that you can either set it from that layer or, as last resort,
    // its DoInitialize method (executes at Simulator::Run) will get it from the node itself.
    // Some parts of the simulator probably depend on the same fallback.
    // So it's a better method to set it with node->AggregateObject(mobility)
    robot->AggregateObject(robotMobilityModel);

    // =========================================================================
    // 3. MobilityModel for both the Coordinator and EndDevices (Static)
    // =========================================================================
    // IMPORTANT: First position of the CSV is coordinator's position
    std::ifstream positionsFile(nodePositions);
    if (!positionsFile.is_open()) {
        NS_FATAL_ERROR("Error: Could not open the node positions file: " << nodePositions);
    }

    uint32_t nodeIndex = 0;
    uint32_t totalNodes = nodes.GetN();

    while (std::getline(positionsFile, line)) {
        // Ignore empty lines or comments (lines starting with '#')
        if (line.empty() || line[0] == '#') {
            continue;
        }

        if (nodeIndex >= totalNodes - 1) {
            NS_LOG_WARN("CSV contains more positions than available static nodes. Ignoring extra rows.");
            break;
        }

        std::stringstream ss(line);
        std::string item;
        double x = 0.0, y = 0.0, z = 0.0;

        // Parse the X, Y, Z coordinates
        if (std::getline(ss, item, ',')) x = std::stod(item);
        if (std::getline(ss, item, ',')) y = std::stod(item);
        if (std::getline(ss, item, ',')) z = std::stod(item);

        // Create and configure the ConstantPositionMobilityModel
        Ptr<ConstantPositionMobilityModel> staticMobility = CreateObject<ConstantPositionMobilityModel>();
        staticMobility->SetPosition(Vector(x, y, z));

        // Assign the mobility model directly to the node via aggregation
        nodes.Get(nodeIndex)->AggregateObject(staticMobility);

        nodeIndex++;
    }

    positionsFile.close();

    // Sanity check: Ensure the CSV provided exactly N-1 positions
    if (nodeIndex < totalNodes - 1) {
        NS_FATAL_ERROR("Error: The CSV file contains fewer positions (" << nodeIndex 
                       << ") than the required static nodes (" << totalNodes - 1 << ").");
    }

    // =========================================================================
    // 5. Instantiate the Data Collector and Connect MAC SAP Callbacks
    // =========================================================================   
    // 1. Create the instance of our collector class.
    // This will open the CSV file and write the headers immediately.
    SlamDataCollector collector("slam_dataset_run1.csv");

    uint32_t totalDevices = devices.GetN();

    // 2. Iterate through all devices in the PAN to connect their MAC callbacks
    for (uint32_t i = 0; i < totalDevices; i++) {
        // Retrieve the MAC layer for the current device
        Ptr<LrWpanMac> macLayer = devices.Get(i)->GetObject<LrWpanNetDevice>()->GetMac();

        if (i < totalDevices - 1) {
            // Static Nodes (Coordinator + End Devices): Index 0 to N-2
            // They are transmitters. We connect the Confirm callback to monitor Tx status.
            macLayer->SetMcpsDataConfirmCallback(
                MakeCallback(&SlamDataCollector::OnDataConfirm, &collector)
            );
        } else {
            // Mobile Robot: Index N-1 (The last device in the container)
            // It is a passive listener. We connect the Indication callback to capture RSSI.
            macLayer->SetMcpsDataIndicationCallback(
                MakeCallback(&SlamDataCollector::OnDataIndication, &collector)
            );
        }
    }

    // =========================================================================
    // 6. Schedule Data Transmissions (McpsDataRequest)
    // =========================================================================
    // Setup the common parameters for a broadcast transmission
    McpsDataRequestParams params;
    params.m_srcAddrMode = SHORT_ADDR;
    params.m_dstAddrMode = SHORT_ADDR;
    params.m_dstPanId = PAN_ID;
    params.m_dstAddr = Mac16Address(BROADCAST_ADDR); // IEEE 802.15.4 Broadcast Address
    params.m_msduHandle = 0;                  // Will be dynamically updated
    params.m_txOptions = TX_OPTION_NONE;      // No ACK required for Broadcast

    double txInterval = 0.5; // Each static node transmits twice per second (every 500 ms)
    uint32_t numTransmitters = totalNodes - 1; // 9 nodes (Index 0 to 8)

    // Iterate through all static transmitters
    for (uint32_t i = 0; i < numTransmitters; i++) {
        
        // Retrieve the MAC layer pointer for the current transmitter
        Ptr<LrWpanMac> macLayer = devices.Get(i)->GetObject<LrWpanNetDevice>()->GetMac();

        // Stagger transmissions by 50ms per node to prevent CSMA/CA collisions
        // e.g., Node 0 at 0ms, Node 1 at 50ms, Node 2 at 100ms...
        Time staggerOffset = MilliSeconds(i * 50);

        uint32_t seqNum = 0; // Unique sequence ID for this specific node

        // Schedule transmissions from t=0.0 up to the end of the robot's trajectory
        for (double t = 0.0; t <= lastTrajectoryTime; t += txInterval) {
            
            // 1. Encode the sequence number into a 4-byte buffer (Big Endian format)
            // This is the "ID" we will decode in the DataIndication callback
            uint8_t buffer[4];
            buffer[0] = (seqNum >> 24) & 0xFF;
            buffer[1] = (seqNum >> 16) & 0xFF;
            buffer[2] = (seqNum >> 8) & 0xFF;
            buffer[3] = seqNum & 0xFF;

            // 2. Create the packet containing our encoded payload
            Ptr<Packet> p = Create<Packet>(buffer, 4);

            // 3. Update the handle (Useful if debugging MAC traces)
            // It must be an 8-bit integer, so we wrap it using modulo 255
            params.m_msduHandle = seqNum % 255; 

            // 4. Calculate the exact absolute time this packet should be fired
            Time txTime = Seconds(t) + staggerOffset;

            // 5. Inject the event into the NS-3 Scheduler
            Simulator::Schedule(txTime, 
                                &LrWpanMac::McpsDataRequest, 
                                macLayer, 
                                params, 
                                p);

            seqNum++; // Increment for the next transmission
        }
    }




    

}