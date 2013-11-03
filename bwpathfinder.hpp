#ifndef __BWPATHFINDER_HPP__
#define __BWPATHFINDER_HPP__

#include <vector>
#include <set>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

namespace bwpathfinder {

    struct Node;
    typedef boost::shared_ptr<Node> NodePtr;
    struct Link;
    typedef boost::shared_ptr<Link> LinkPtr;
    class TrafficPattern;
    typedef boost::shared_ptr<TrafficPattern> TrafficPatternPtr;
    class Network;
    typedef boost::shared_ptr<Network> NetworkPtr;
    struct Path;
    typedef boost::shared_ptr<Path> PathPtr;
    class Pathfinder;
    typedef boost::shared_ptr<Pathfinder> PathfinderPtr;

    struct smart_ptr_less_than
    {
        template< typename T >
        bool operator()( T p1, T p2 ) const {
            return p1.get() < p2.get();
        }
    }; 

    struct Link {
        NodePtr a, b;
        float bandwidth;
        float latency;
        int   maximum_paths;

        Link(NodePtr a, NodePtr b, float bandwidth, float latency, int maximum_paths=-1) : 
            a(a),
            b(b),
            bandwidth(bandwidth),
            latency(latency),
            maximum_paths(-1) {
        }

        // Used for pathfinder algo
        std::set<PathPtr, smart_ptr_less_than> paths;
        mutable float bwRequested;
        mutable float historyPenalty;

        std::vector<PathPtr> getPaths() {
            return std::vector<PathPtr>(paths.begin(), paths.end());
        }

        float costToUse(PathPtr path) const;

        float bwShareW(float bw) const {
            float bwAll = bw + this->bwRequested;
            if (bwAll < this->bandwidth)
                return bw;
            return (bw / bwAll) * this->bandwidth;
        }

        float bwShare(float bw) const {
            float bwAll = this->bwRequested;
            if (bwAll < this->bandwidth)
                return bw;
            return (bw / bwAll) * this->bandwidth;
        }

        void recomputeHistoryPenalty(float increment) {
            if (solutionPartialCost() > 0.0) {
                this->historyPenalty += this->bandwidth * increment;
            }
        }

        float solutionPartialCost() const {
            // printf("spc: %e %e %e %lu\n", this->bandwidth,
                // this->bwRequested, this->historyPenalty, this->paths.size());
            if (this->bwRequested < this->bandwidth)
                return 0.0;
            return this->bwRequested - this->bandwidth;
        }
    };

    struct Node {
        static size_t id_counter;
        size_t id;
        float latency;
        std::string label;

        Node(float latency) : 
            latency(latency) { 
            this->id = id_counter++;
        }
    };

    struct Path : public boost::enable_shared_from_this<Path>{
        NodePtr src;
        NodePtr dst;
        float requested_bw;
        float delivered_bw;
        std::vector<LinkPtr> path;

        Path() :
            requested_bw(-1),
            delivered_bw(-1) {
        }

        void assign(std::vector<LinkPtr> newPath) {
            this->path = newPath;
            if (this->src == this->dst)
                return;

            assert(newPath.size() > 0);

            // Make sure the path begins and ends at out src, dst
            auto fst = this->path.front();
            assert(fst->a == this->src || fst->b == this->src);
            auto lst = this->path.back();
            assert(lst->a == this->dst || lst->b == this->dst);

            for (auto link : path) {
                link->paths.insert(shared_from_this());
                link->bwRequested += this->requested_bw;
            }
        }

        void ripup() {
            for (auto link : path) {
                auto f = link->paths.find(shared_from_this());
                assert(f != link->paths.end());
                link->paths.erase(f);
                link->bwRequested -= this->requested_bw;
            }
            path.clear();
        }

        float calcCost(float hopCost) {
            float cost = 0.0;
            for (auto link : path) {
                cost = std::max(cost, this->requested_bw - link->bwShare(this->requested_bw));
            }
            cost += path.size() * hopCost;
            return cost;
        }

        bool operator==(Path const& p) const {
            return src == p.src &&
                    dst == p.dst &&
                    delivered_bw == p.delivered_bw &&
                    path.size() == p.path.size() &&
                    path == p.path;
        }
        bool operator!=(Path const& p) const {
            return !((*this) == p);
        }
    };


    class NetworkSimulation;
    class Network : public boost::enable_shared_from_this<Network> {
        friend class NetworkSimulation;
        friend class Pathfinder;
        std::set<NodePtr, smart_ptr_less_than> nodes;
        std::set<LinkPtr, smart_ptr_less_than> links;
        std::set<PathPtr, smart_ptr_less_than> paths;

        std::map<NodePtr, std::set<LinkPtr, smart_ptr_less_than>, smart_ptr_less_than> linkIndex;

    public:
        unsigned long flit_size_in_bytes;

        Network(unsigned long flit_size_in_bytes):
            flit_size_in_bytes(flit_size_in_bytes) {
        }

        void addLink(NodePtr src, NodePtr dst, float bw, float latency = -1) {
            // printf("[%p] addLink %s %s %e %e\n", this, src->label.c_str(),
                // dst->label.c_str(), bw, latency);
            nodes.insert(src);
            nodes.insert(dst);

            LinkPtr link(new Link(src, dst, bw, latency));
            links.insert(link);
            linkIndex[src].insert(link);
            linkIndex[dst].insert(link);
        }

        void addPath(PathPtr path) {
            // printf("[%p] addPath: %p %p %f\n", this, path->src.get(), path->dst.get(), path->requested_bw);
            assert(path->dst != NodePtr());
            assert(path->src != NodePtr());
            assert(nodes.find(path->dst) != nodes.end());
            assert(nodes.find(path->src) != nodes.end());
            for (auto link : path->path) {
                assert(link != LinkPtr());
                assert(links.find(link) != links.end());
            }
            this->paths.insert(path);
        }

        std::vector<PathPtr> getPaths () {
            return std::vector<PathPtr>(paths.begin(), paths.end());
        }

        LinkPtr findLink(NodePtr src, NodePtr dst) {
            // printf("findLink %p %p\n", src.get(), dst.get());
            auto flinkSet = linkIndex.find(src);
            if (flinkSet == linkIndex.end()) {
                printf("No link set\n");
                return LinkPtr();
            }
            auto& linkSet = flinkSet->second;
            for (auto link: linkSet) {
                if (dst == NodePtr())
                    return link;
                // printf("\t%p %p\n", link->a.get(), link->b.get());
                if ((link->a == src && link->b == dst) ||
                    (link->a == dst && link->b == src))
                    return link;
            }
            return LinkPtr();
        }

        void simulateDeliveredBandwidth();
    };

    class Pathfinder {
        NetworkPtr network;

        float iterate();
        float solutionCost();

    public:
        float hopCost;
        uint64_t iteration;
        float cost;
        Pathfinder() { }

        void init(NetworkPtr network, float hopCost) {
            this->network = network;
            this->hopCost = hopCost;
            this->iteration = 0;
            cost = std::numeric_limits<float>::infinity();

            for (PathPtr path : this->network->paths) {
                path->path.clear();
            }

            // Zero all state variables
            for (LinkPtr link : this->network->links) {
                link->paths.clear();
                link->bwRequested = 0.0;
                link->historyPenalty = 0.0;
            }
        }

        float solve(float desiredCost, uint64_t maxIterations);
        float solveConverge(float improvementThreshold, uint64_t maxIterations);
    };


}

#endif // __BWPATHFINDER_HPP__