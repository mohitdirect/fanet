/*
 * traffic.h - Traffic Pattern Setup Functions
 */

#ifndef MYFANET_TRAFFIC_H
#define MYFANET_TRAFFIC_H

#include <cstdint>

namespace myfanet {

// Setup aircraft communication pairs with ACKs
void SetupAircraftCommunication(uint32_t numNodes, double dataRate, double ackRate);

} // namespace myfanet

#endif // MYFANET_TRAFFIC_H
