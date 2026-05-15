// rssi_sample.h
#pragma once // Evita que se incluya dos veces
#include <cstdint>

struct RssiSample {
    double rssi;
    double tMuestra;
    uint32_t ed;

    RssiSample() : rssi(0.0), tMuestra(0.0), ed(0) {}
    RssiSample(double r, double t, uint32_t e) : rssi(r), tMuestra(t), ed(e) {}
};