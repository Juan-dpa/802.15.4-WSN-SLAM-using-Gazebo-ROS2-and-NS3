// SlamDataCollector.h
#ifndef SLAM_DATA_COLLECTOR_H // Guardas para evitar inclusiones dobles
#define SLAM_DATA_COLLECTOR_H

#include "ns3/lr-wpan-mac.h"
#include "ns3/packet.h"
#include <fstream>
#include <string>

using namespace ns3;
using namespace ns3::lrwpan;

class SlamDataCollector {
public:
    SlamDataCollector(std::string csvFilename);
    ~SlamDataCollector();

    void OnDataConfirm(McpsDataConfirmParams params);
    void OnDataIndication(McpsDataIndicationParams params, Ptr<Packet> p);

private:
    std::ofstream m_csvFile;
    uint32_t m_rxPackets;
    uint32_t m_txFailures;
};

#endif