#ifndef __NETWORK_SIMULATION_HPP__
#define __NETWORK_SIMULATION_HPP__

#include "bwpathfinder.hpp"
#include "libsimfw/simfw.hpp"
#include "libsimfw/timer.hpp"

#include <map>

namespace bwpathfinder {
    typedef simfw::TimeInPS Time;

    class NetworkSimulation : public simfw::Simulation<Time>
    {
        struct Flit {
            PathPtr owner;
            size_t hops;
            NodePtr lastNode;

            Flit(PathPtr path) :
                owner(path),
                hops(0),
                lastNode(path->src) {
            }

            NodePtr nextNode() {
                if (arrived())
                    return owner->dst;
                assert(hops < owner->path.size());
                LinkPtr currLink = owner->path[hops];
                // printf("nextNode src %p dst %p curr (%p %p)\n",
                        // owner->src.get(), owner->dst.get(), currLink->a.get(), currLink->b.get());
                if (currLink->a == lastNode)
                    return currLink->b;
                else if (currLink->b == lastNode)
                    return currLink->a;
                else
                    assert(false && "Invalid lastNode, currLink association");
            }

            void advanceHop() {
                lastNode = nextNode();
                hops += 1;
            }

            bool arrived() {
                return lastNode == owner->dst;
            }
        };

        class SimulatedLink;

        class SimulatedNode : public simfw::InputPort<Time, Flit*>
        {
            std::map<uint64_t, SimulatedLink*> outPorts;

        public:
            NodePtr node;
            std::map<PathPtr, uint64_t> flits_seen;

            SimulatedNode(NetworkSimulation* sim, NodePtr node) :
                simfw::InputPort<Time, Flit*>(sim),
                node(node) {
            }

            void notifyLink(NodePtr toNode, SimulatedLink* link) {
                outPorts[toNode->id] = link;
            }

            void inject(Time time, Flit* f, SimulatedLink* from) {
                f->advanceHop();
                flits_seen[f->owner] += 1;
                if (f->arrived()) {
                    // We've arrived!
                    delete f;
                } else {
                    NodePtr next = f->nextNode();
                    outPorts[next->id]->inject(time, f);
                }
            }
        };

        class SimulatedLink : public simfw::InputPort<Time, Flit*>
        {
            LinkPtr link;
            size_t max_in_flight;
            Time latency;
            size_t bufferedItems;
            size_t roundRobinIndex;
            std::map<Node*, std::deque<Flit*> > buffers;
            std::set<Flit*> inFlight;
            SimulatedNode* nodeA;
            SimulatedNode* nodeB;

        public:
            uint64_t flitsDelivered;

            SimulatedLink(NetworkSimulation* sim, LinkPtr link) :
                simfw::InputPort<Time, Flit*>(sim),
                link(link),
                bufferedItems(0),
                roundRobinIndex(0),
                flitsDelivered(0) {
                float max_iff = link->bandwidth * link->latency / sim->flit_size_in_bytes;
                this->max_in_flight = round(max_iff);
                if (this->max_in_flight < 1) {
                    printf("Config violation: max in flight (%f) < 1!\n", max_iff);
                    printf("\tbw: %e  latency: %e  flit_size: %lu bytes\n",
                        link->bandwidth, link->latency, sim->flit_size_in_bytes);
                    assert(false);
                }
                this->latency = link->latency;
                this->nodeA = sim->getNode(link->a);
                this->nodeB = sim->getNode(link->b);
                assert(nodeA != NULL);
                assert(nodeB != NULL);
            }

            ~SimulatedLink() {
                for (auto flit : inFlight) {
                    delete flit;
                }

                for (auto& buffer : buffers) {
                    for (auto flit : buffer.second) {
                        delete flit;
                    }
                }
            }

            void inject(Time time, Flit* f) {
                if (inFlight.size() >= max_in_flight) {
                    buffers[f->owner->src.get()].push_back(f);
                    bufferedItems += 1;
                } else {
                    inFlight.insert(f);
                    this->deliverIn(latency, f);
                }
            }

            virtual void recieve(Time time, Flit* f) {
                inFlight.erase(f);
                this->flitsDelivered += 1;
                if (bufferedItems > 0) {
                    // Find next to send according to RR policy
                    auto iter = buffers.begin();
                    for (size_t i=0; i<roundRobinIndex; i++) {
                        iter++;
                    }

                    const size_t num_buffers = buffers.size();
                    // if (num_buffers > 1)
                        // printf("[%p] bi: %lu, num_buffers: %lu\n", this, bufferedItems, num_buffers);
                    for (size_t i=0; i<num_buffers; i++) {
                        if (!iter->second.empty()) {
                            Flit* inj_flit = iter->second.front();
                            iter->second.pop_front();

                            inFlight.insert(inj_flit);
                            this->deliverIn(latency, inj_flit);
                            roundRobinIndex = (i + roundRobinIndex + 1) % num_buffers;
                            // if (num_buffers > 1)
                                // printf("rrI: %lu\n", roundRobinIndex);
                            bufferedItems -= 1;
                            break;
                        } else {
                            iter++;
                            if (iter == buffers.end())
                                iter = buffers.begin();
                        }
                    }
                }

                NodePtr next = f->nextNode();
                // printf("next: %p   a %p b %p\n",
                    // next.get(), nodeA->node.get(), nodeB->node.get());
                // printf("link : %p %p\n", link->a.get(), link->b.get());
                if (next == nodeA->node)
                    nodeA->inject(time, f, this);
                else if (next == nodeB->node)
                    nodeB->inject(time, f, this);
                else
                    assert(false && "Invalid route found!");
            }
        };

        class InjectionPort : public simfw::Timer<Time>
        {
            PathPtr path;
            SimulatedLink* link;
        public:
            InjectionPort(NetworkSimulation* sim, Time period,
                          PathPtr path, SimulatedLink* link) :
                simfw::Timer<Time>(sim, period),
                path(path),
                link(link) {
            }

        protected:
            virtual bool ding(uint64_t i) {
                link->inject(simulation->now(), new Flit(path));
                // printf("Injecting flit %lf %lu\n", simulation->now().seconds(), i);
                return true;
            }
        };

        NetworkPtr network;
        size_t flit_size_in_bytes;
        Time longestPeriod;
        Time slowestClock;
        size_t longestPath;
        std::map<PathPtr, InjectionPort*, smart_ptr_less_than> injectionPorts;
        std::map<NodePtr, SimulatedNode*, smart_ptr_less_than> simulatedNodes;
        std::map<LinkPtr, SimulatedLink*, smart_ptr_less_than> simulatedLinks;

    public: 
        NetworkSimulation(NetworkPtr network) {
            this->network = network;
        }

        void clear() {
            for (auto p : injectionPorts) {
                delete p.second;
            }
            injectionPorts.clear();
            simulatedNodes.clear();
            simulatedLinks.clear();
        }

        void init() {
            clear();
            this->flit_size_in_bytes = network->flit_size_in_bytes;

            slowestClock = 0.0;
            for(auto node : network->nodes) {
                simulatedNodes.insert(std::make_pair(node, new SimulatedNode(this, node)));
                if (slowestClock < node->latency)
                    slowestClock = node->latency;
            }

            for(auto link: network->links) {
                if (slowestClock < link->latency)
                    slowestClock = link->latency;
                simulatedLinks.insert(std::make_pair(link, new SimulatedLink(this, link)));
                SimulatedLink* slink = simulatedLinks.find(link)->second;
                SimulatedNode* n;

                n = getNode(link->a);
                assert(n != NULL);
                n->notifyLink(link->b, slink);

                n = getNode(link->b);
                assert(n != NULL);
                n->notifyLink(link->a, slink);
            }

            Time largestPeriod = 0.0;
            this->longestPath = 0;
            for (auto path: network->paths){
                Time period = 1.0 / (path->requested_bw / flit_size_in_bytes);
                if (largestPeriod < period)
                    largestPeriod = period;
                if (path->path.size() > longestPath)
                    longestPath = path->path.size();
                LinkPtr vlink = network->findLink(path->src, NodePtr());
                assert(vlink != LinkPtr());
                auto flink = simulatedLinks.find(vlink);
                assert(flink != simulatedLinks.end());
                SimulatedLink* slink = flink->second;
                injectionPorts.insert(std::make_pair(path, new InjectionPort(this, period, path, slink)));
            }

            this->longestPeriod = largestPeriod;
        }

        ~NetworkSimulation() {
            for (auto pr : injectionPorts) {
                delete pr.second;
            }

            for (auto pr : simulatedNodes) {
                delete pr.second;
            }

            for (auto pr : simulatedLinks) {
                delete pr.second;
            }
        }

        SimulatedNode* getNode(NodePtr n) {
            auto fnode = simulatedNodes.find(n);
            if (fnode == simulatedNodes.end())
                return NULL;
            return fnode->second;
        }

        void simulate() {
            init();

            // Simulate N of the slowest packet injections
            float simTime = longestPeriod.seconds() * 100.0 +
                          10 * longestPath * slowestClock.seconds();
            this->goUntil(simTime);
            this->network->simulatedTime = simTime;

            for(auto node : network->nodes) {
                auto snode = simulatedNodes[node];
                uint64_t flitsDelivered = 0;
                for (auto p : snode->flits_seen) {
                    flitsDelivered += p.second;
                }
                node->simulatedFlitsDelivered = flitsDelivered;
            }

            for(auto link: network->links) {
                link->simulatedFlitsDelivered = simulatedLinks[link]->flitsDelivered;
            }
        }

        void setDeliveredBandwidths() {
            float runtime = this->now().seconds();
            for (auto path: this->network->paths) {
                SimulatedNode* dst = this->getNode(path->dst);
                assert(dst != NULL);
                uint64_t recvd_flits = dst->flits_seen[path];
                uint64_t bytes_recvd = recvd_flits * this->flit_size_in_bytes;
                path->delivered_bw = ((float)bytes_recvd) / runtime;
            }
        }
    };
}

#endif // __NETWORK_SIMULATION_HPP__