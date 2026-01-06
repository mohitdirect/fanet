/*
 * cr_protocols.h - Cognitive Radio Protocol Implementations
 */

#ifndef MYFANET_CR_PROTOCOLS_H
#define MYFANET_CR_PROTOCOLS_H

namespace myfanet {

// CR-ED: Reactive energy-detection loop
void CrEdLoop();

// CR-SCAN: Periodic scanning loop
void CrScanLoop();

// CR-COOP: Cooperative sensing loop (quorum)
void CrCoopLoop();

} // namespace myfanet

#endif // MYFANET_CR_PROTOCOLS_H
