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
#include "ns3/buildings-module.h"
#include "ns3/waypoint-mobility-model.h"
#include "ns3/waypoint.h"

// FROM C++

#include <vector>
#include <fstream>
#include <iostream>

#include "SlamDataCollector.h"
#include "CsvScenarioHelper.h"

// DEFINES

#define PAN_ID 0xAAAA
#define NUMBER_COORDINATORS 1
#define NUMBER_EDS 8
#define NUMBER_ROBOTS 1
#define BROADCAST_ADDR "ff:ff"
#define SIMULATION_TIME_BUFFER 1
#define TX_INTERVAL 0.5

using namespace ns3;
using namespace ns3::lrwpan;

NS_LOG_COMPONENT_DEFINE ( "RSSI-TOPOLOGY" );

int
main(int argc, char* argv[])
{

    Time::SetResolution(Time::PS);
    std::string trajectoryFilename = "scratch/Proyecto_ROS2_WSN/Inputs/trajectory.csv";
    std::string nodePositions = "scratch/Proyecto_ROS2_WSN/Inputs/positions.csv";
    std::string buildingFile = "scratch/Proyecto_ROS2_WSN/Inputs/buildings.csv";
    std::string outputFile = "scratch/Proyecto_ROS2_WSN/Outputs/slam_dataset_run1.csv";
    bool pcapTracing = false;

    CommandLine parameters;
    parameters.AddValue("pcap", "Flag for generating pcap of traffic",pcapTracing);
    parameters.AddValue("trajectoryFilename","Filename of Mobility Data",trajectoryFilename);
    parameters.AddValue("nodePositions","Filename of Mobility Data",nodePositions);
    parameters.AddValue("buildingFile","Filename of Buildings Data",buildingFile);
    parameters.AddValue("outputFile", "Filename of outputFile", outputFile);
    parameters.Parse(argc,argv);

    // Helper Configuration
    LrWpanHelper lrWpanHelper;
    lrWpanHelper.SetPropagationDelayModel("ns3::ConstantSpeedPropagationDelayModel");
    lrWpanHelper.AddPropagationLossModel("ns3::HybridBuildingsPropagationLossModel");

    NodeContainer nodes;
    nodes.Create(NUMBER_COORDINATORS+NUMBER_EDS+NUMBER_ROBOTS); 

    
    NetDeviceContainer devices = lrWpanHelper.Install(nodes);

    if(pcapTracing==true){
        // Enable PCAP for Robot
        lrWpanHelper.EnablePcap("scratch/Proyecto_ROS2_WSN/Pcap/pcap", devices.Get(devices.GetN()-1), true);
    }

    // Set Robot in Sniffer Moder
    Ptr<LrWpanMac> robotMac = devices.Get(devices.GetN() - 1)->GetObject<LrWpanNetDevice>()->GetMac();
    robotMac->m_macPromiscuousMode = true;
    
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
    double lastTrajectoryTime = CsvScenarioHelper::LoadTrajectory(trajectoryFilename,robotMobilityModel, robot);
    
    // =========================================================================
    // 3. MobilityModel for both the Coordinator and EndDevices (Static)
    // =========================================================================
    CsvScenarioHelper::LoadStaticPositions(nodePositions, nodes);

    // =========================================================================
    // 4. BuildingData for LossModel
    // =========================================================================
    CsvScenarioHelper::LoadBuildings(buildingFile);
    BuildingsHelper buildHelp;
    buildHelp.Install(nodes);

    // =========================================================================
    // 5. Instantiate the Data Collector and Connect MAC SAP Callbacks
    // =========================================================================   
    // 1. Create the instance of our collector class.
    // This will open the CSV file and write the headers immediately.
    SlamDataCollector collector(outputFile);
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
    params.m_dstAddr = Mac16Address(devices.Get(0)->GetObject<LrWpanNetDevice>()->GetMac()->GetShortAddress()); // Coordinator MAC address
    params.m_msduHandle = 0;                  // Will be dynamically updated 
    params.m_txOptions = TX_OPTION_NONE;      // No ACK required for Broadcast

    // ==============================================================================
    // STOCHASTIC MODELS INITIALIZATION (Parametrized for real IoT Hardware)
    // Instantiated outside the loop to prevent memory leaks and optimize execution
    // ==============================================================================

    // 1. Clock Drift / Skew (Hardware derivation specific to the node)
    // Distribution: Normal, Mean = 0, StdDev = 10 ppm (Variance = 1e-10)
    Ptr<NormalRandomVariable> clockSkewVar = CreateObject<NormalRandomVariable>();
    clockSkewVar->SetAttribute("Mean", DoubleValue(0.0));
    clockSkewVar->SetAttribute("Variance", DoubleValue(1e-10)); 

    // 2. Initial Phase Offset (Boot Jitter)
    // Distribution: Uniform, 0 to 5 ms. 
    // Simulates the static delay when the RTOS initializes the TDMA timer at boot.
    Ptr<UniformRandomVariable> phaseOffsetVar = CreateObject<UniformRandomVariable>();
    phaseOffsetVar->SetAttribute("Min", DoubleValue(0.0));
    phaseOffsetVar->SetAttribute("Max", DoubleValue(0.005));

    // 3. OS/MAC Task Scheduling Jitter (Per-packet latency)
    // Distribution: Uniform, 0 to 2 ms.
    // Simulates context-switching and interrupt latency before Tx.
    Ptr<UniformRandomVariable> osJitterVar = CreateObject<UniformRandomVariable>();
    osJitterVar->SetAttribute("Min", DoubleValue(0.0));
    osJitterVar->SetAttribute("Max", DoubleValue(0.002));

    // ==============================================================================
    // TRANSMISSION SCHEDULING LOOP
    // ==============================================================================

    // Iterate through all static transmitters
    for (uint32_t i = 0; i < NUMBER_EDS; i++) {
        
        // Retrieve the MAC layer pointer for the current transmitter
        Ptr<LrWpanMac> macLayer = devices.Get(i)->GetObject<LrWpanNetDevice>()->GetMac();

        // Engineer's Perfect TDMA Stagger (e.g., Node 0 at 0ms, Node 1 at 50ms...)
        double baseStagger = i * 0.050; // Represented directly in seconds

        // -- PER-NODE STOCHASTICS (Calculated once for the entire life of this node) --
        
        // a) Get the unique crystal defect for this node
        double alpha_i = clockSkewVar->GetValue();
        // Truncate to realistic commercial limits (+/- 30 ppm) to avoid statistical outliers
        if (alpha_i > 30e-6) alpha_i = 30e-6;
        if (alpha_i < -30e-6) alpha_i = -30e-6;

        // b) Get the static initialization delay for this node's timer
        double phi_i = phaseOffsetVar->GetValue();

        uint32_t seqNum = 0; // Unique sequence ID for this specific node
        double t_nominal = 0.0; // The theoretical time in a perfect world

        // Schedule transmissions from t=0.0 up to the end of the robot's trajectory
        while (t_nominal <= lastTrajectoryTime) {
            
            // -- PER-PACKET STOCHASTIC --
            // c) Get the dynamic OS delay for this specific packet
            double jitter_k = osJitterVar->GetValue();

            // ----------------------------------------------------------------------
            // Absolute Real Time = (Absolute Nominal Time * Hardware Skew) + Phase Offset + OS Jitter
            // ----------------------------------------------------------------------
            double absoluteNominalTime = t_nominal + baseStagger;
            double txTimeReal = (absoluteNominalTime * (1.0 + alpha_i)) + phi_i + jitter_k;

            // Ensure the deformed time doesn't schedule events past our simulation end
            if (txTimeReal <= lastTrajectoryTime) {
                
                // 1. Encode the sequence number into a 4-byte buffer (Big Endian format)
                uint8_t buffer[4];
                buffer[0] = (seqNum >> 24) & 0xFF;
                buffer[1] = (seqNum >> 16) & 0xFF;
                buffer[2] = (seqNum >> 8) & 0xFF;
                buffer[3] = seqNum & 0xFF;

                // 2. Create the packet containing our encoded payload
                Ptr<Packet> p = Create<Packet>(buffer, 4);

                // 3. Update the handle (Useful if debugging MAC traces)
                params.m_msduHandle = seqNum % 255; 

                // 4. Inject the event into the NS-3 Scheduler using the computed real time
                Simulator::Schedule(Seconds(txTimeReal), 
                                    &LrWpanMac::McpsDataRequest, 
                                    macLayer, 
                                    params, 
                                    p);
            }

            seqNum++; // Increment for the next transmission
            t_nominal += TX_INTERVAL; // Advance the theoretical loop by the cycle time
        }
    }

    Simulator::Stop(Seconds(lastTrajectoryTime+SIMULATION_TIME_BUFFER));
    Simulator::Run();

    Simulator::Destroy();
    return 0;

}