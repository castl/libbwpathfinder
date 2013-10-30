#include "bwpathfinder.hpp"
#include "network_simulation.hpp"

#include <cstdio>

namespace bwpathfinder {

    size_t Node::id_counter = 0;

    void Network::simulateDeliveredBandwidth() {
        NetworkSimulation sim(shared_from_this());
        sim.simulate();
        sim.setDeliveredBandwidths();
        sim.flushMessages();
    }
}
