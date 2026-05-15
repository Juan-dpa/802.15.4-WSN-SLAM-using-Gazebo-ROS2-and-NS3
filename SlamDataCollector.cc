#include "SlamDataCollector.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <iostream>

// Constructor
SlamDataCollector::SlamDataCollector(std::string csvFilename) 
    : m_rxPackets(0), m_txFailures(0) 
{
    m_csvFile.open(csvFilename);
    if (!m_csvFile.is_open()) {
        NS_FATAL_ERROR("Error: Could not create the output file: " << csvFilename);
    }
    m_csvFile << "Time_s,Src_MAC,Seq_Num,RSSI_dBm\n";
}

// Destructor
SlamDataCollector::~SlamDataCollector() 
{
    if (m_csvFile.is_open()) {
        m_csvFile.close();
    }
    std::cout << "\n[SLAM Collector] Simulation finished." << std::endl;
    std::cout << "- SLAM packets received: " << m_rxPackets << std::endl;
    std::cout << "- Channel access failures (CSMA/CA): " << m_txFailures << std::endl;
}

// SAP::DataConfirm Callback method
void SlamDataCollector::OnDataConfirm(McpsDataConfirmParams params) 
{
    if (params.m_status != MacStatus::SUCCESS) {
        m_txFailures++;
    }
}
// SAP::DataIndication Callback method
void SlamDataCollector::OnDataIndication(McpsDataIndicationParams params, Ptr<Packet> p) 
{
    uint8_t buffer[4];
    p->CopyData(buffer, 4);
    uint32_t seqNum = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];

    double rxTime = Simulator::Now().GetSeconds();
    int8_t rssi = params.m_rssi;
    Mac16Address srcAddr = params.m_srcAddr;

    m_csvFile << rxTime << "," << srcAddr << "," << seqNum << "," << static_cast<int>(rssi) << "\n";
    m_rxPackets++;
}