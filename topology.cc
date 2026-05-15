//
// Topología de Red WSN sobre 801.15.4  
// para cálculo de SLAM
//
//
// Integrantes: Juan del Pozo Ávila, Ulfer Cit Flores Taco, Sébastien Deurveilher
// Curso 2025-2026 
//



// INCLUDES

// FROM NS3

#include "ns3/core-module.h"
#include "ns3/lr-wpan-module.h"

// FROM C++

#include <vector>
#include <fstream>
#include <iostream>

#include "entities.h"

// DEFINES

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ( "RSSI-TOPOLOGY" );

