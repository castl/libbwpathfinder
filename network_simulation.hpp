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
            Path* owner;
            size_t hops;
            Node* lastNode;

            Flit(Path* path) :
                owner(path),
                hops(0),
                lastNode(path->src.get()) {
            }

            Node* nextNode() {
                if (arrived())
                    return owner->dst.get();
                assert(hops < owner->path.size());
                Link* currLink = owner->path[hops].get();
                // printf("nextNode src %p dst %p curr (%p %p)\n",
                        // owner->src.get(), owner->dst.get(), currLink->a.get(), currLink->b.get());
                if (currLink->a.get() == lastNode)
                    return currLink->b.get();
                else if (currLink->b.get() == lastNode)
                    return currLink->a.get();
                else
                    assert(false && "Invalid lastNode, currLink association");
            }

            void advanceHop() {
                lastNode = nextNode();
                hops += 1;
            }

            bool arrived() {
                return lastNode == owner->dst.get();
            }
        };

        class SimulatedLink;

        class SimulatedNode : public simfw::InputPort<Time, Flit*>
        {
            std::map<uint64_t, SimulatedLink*> outPorts;

        public:
            Node* node;
            std::map<Path*, uint64_t> flits_seen;

            SimulatedNode(NetworkSimulation* sim, Node* node) :
                simfw::InputPort<Time, Flit*>(sim),
                node(node) {
            }

            virtual ~SimulatedNode() { }

            void notifyLink(Node* toNode, SimulatedLink* link) {
                outPorts[toNode->id] = link;
            }

            void inject(Time time, Flit* f, SimulatedLink* from) {
                f->advanceHop();
                flits_seen[f->owner] += 1;
                if (f->arrived()) {
                    // We've arrived!
                    delete f;
                } else {
                    Node* next = f->nextNode();
                    outPorts[next->id]->inject(time, f);
                }
            }
        };

        class SimulatedLink : public simfw::InputPort<Time, Flit*>
        {
            Link* link;
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

            SimulatedLink(NetworkSimulation* sim, Link* link) :
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
                this->nodeA = sim->getNode(link->a.get());
                this->nodeB = sim->getNode(link->b.get());
                assert(nodeA != NULL);
                assert(nodeB != NULL);
            }

            virtual ~SimulatedLink() {
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

                Node* next = f->nextNode();
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
            Path* path;
            SimulatedLink* link;
            uint64_t flitsInjected;
            SimulatedNode* dst;
        public:
            InjectionPort(NetworkSimulation* sim, Time period,
                          Path* path, SimulatedLink* link,
                          SimulatedNode* dst) :
                simfw::Timer<Time>(sim, period),
                path(path),
                link(link),
                flitsInjected(0),
                dst(dst) {
            }

            virtual ~InjectionPort() { }

        protected:
            virtual bool ding(uint64_t i) {
                uint64_t flitsRecieved = dst->flits_seen[path];
                uint64_t flitsInFlight = flitsInjected - flitsRecieved;
                if (flitsInFlight > 1.1 * path->path.size()) {
                    // Too many flits in flight. Throttle
                    return true;
                }
                link->inject(simulation->now(), new Flit(path));
                flitsInjected += 1;
                // printf("Injecting flit %lf %lu\n", simulation->now().seconds(), i);
                return true;
            }
        };

        Network* network;
        size_t flit_size_in_bytes;
        Time longestPeriod;
        Time slowestClock;
        size_t longestPath;
        std::map<Path*, InjectionPort*> injectionPorts;
        std::map<Node*, SimulatedNode*> simulatedNodes;
        std::map<Link*, SimulatedLink*> simulatedLinks;

    public: 
        NetworkSimulation(Network* network) {
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
                simulatedNodes.insert(std::make_pair(node.get(), new SimulatedNode(this, node.get())));
                if (slowestClock < node->latency)
                    slowestClock = node->latency;
            }

            for(auto link: network->links) {
                if (slowestClock < link->latency)
                    slowestClock = link->latency;
                simulatedLinks.insert(std::make_pair(link.get(), new SimulatedLink(this, link.get())));
                SimulatedLink* slink = simulatedLinks.find(link.get())->second;
                SimulatedNode* n;

                n = getNode(link->a.get());
                assert(n != NULL);
                n->notifyLink(link->b.get(), slink);

                n = getNode(link->b.get());
                assert(n != NULL);
                n->notifyLink(link->a.get(), slink);
            }

            Time largestPeriod = 0.0;
            this->longestPath = 0;
            for (auto path: network->paths){
                // Time period = 1.0 / (path->requested_bw / flit_size_in_bytes);
                Time period = 1.0 / (path->requested_bw / flit_size_in_bytes);
                if (largestPeriod < period)
                    largestPeriod = period;
                if (path->path.size() > longestPath)
                    longestPath = path->path.size();
                Link* vlink = network->findLink(path->src, NodePtr()).get();
                assert(vlink != NULL);
                auto flink = simulatedLinks.find(vlink);
                assert(flink != simulatedLinks.end());
                SimulatedLink* slink = flink->second;
                SimulatedNode* dst = simulatedNodes[path->dst.get()];
                injectionPorts.insert(std::make_pair(path.get(),
                    new InjectionPort(this, period, path.get(), slink, dst)));
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

        SimulatedNode* getNode(Node* n) {
            auto fnode = simulatedNodes.find(n);
            if (fnode == simulatedNodes.end())
                return NULL;
            return fnode->second;
        }

        void simulate() {
            init();

            // Simulate N of the slowest packet injections
            float simTime = longestPeriod.seconds() * 200.0 +
                          20 * longestPath * slowestClock.seconds();
            if (simTime > 1e-5) {
                printf("WARNING: Trying to simulate more than 10us!\n");
                fflush(stdout);
            }
            this->goUntil(simTime);
            this->network->simulatedTime = simTime;

            for(auto node : network->nodes) {
                auto snode = simulatedNodes[node.get()];
                uint64_t flitsDelivered = 0;
                for (auto p : snode->flits_seen) {
                    flitsDelivered += p.second;
                }
                node->simulatedFlitsDelivered = flitsDelivered;
            }

            for(auto link: network->links) {
                link->simulatedFlitsDelivered = simulatedLinks[link.get()]->flitsDelivered;
            }
        }

        void setDeliveredBandwidths() {
            float runtime = this->now().seconds();
            for (auto path: this->network->paths) {
                SimulatedNode* dst = this->getNode(path->dst.get());
                assert(dst != NULL);
                uint64_t recvd_flits = dst->flits_seen[path.get()];
                uint64_t bytes_recvd = recvd_flits * this->flit_size_in_bytes;
                path->delivered_bw = ((float)bytes_recvd) / runtime;
            }
        }
    };
}

#endif // __NETWORK_SIMULATION_HPP__
