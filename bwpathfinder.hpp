#ifndef __BWPATHFINDER_HPP__
#define __BWPATHFINDER_HPP__

#include <vector>
#include <set>
#include <boost/shared_ptr.hpp>

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

    struct Link {
        NodePtr a, b;
        float bandwidth;
        float used_bandwidth;
        int   maximum_paths;

        Link(NodePtr a, NodePtr b, float bandwidth, int maximum_paths=-1) : 
            a(a),
            b(b),
            bandwidth(bandwidth),
            maximum_paths(-1) {
        }
    };

    struct Node {
        static size_t id_counter;
        size_t id;

        Node() { 
            this->id = id_counter++;
        }
    };


    struct Path {
        NodePtr src;
        NodePtr dst;
        float requested_bw;
        float delivered_bw;
        std::vector<LinkPtr> path;

        Path() :
            requested_bw(-1),
            delivered_bw(-1) {
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


    class Network {
        std::set<NodePtr> nodes;
        std::set<LinkPtr> links;
        std::set<PathPtr> paths;

    public:
        Network() { }

        void addLink(NodePtr src, NodePtr dst, float bw) {

        }

        void addPath(PathPtr path) {
            this->paths.insert(path);
        }

        std::vector<PathPtr> getPaths () {
            return std::vector<PathPtr>(paths.begin(), paths.end());
        }

        LinkPtr findLink(NodePtr, NodePtr) {
            return LinkPtr();
        }

        void computeDeliveredBandwidth() {

        }
    };

    class Pathfinder {
        void iterate0();
        void iterate();
    public:
        Pathfinder() { }
        Pathfinder(NetworkPtr network) {
            this->init(network);
        }

        void init(NetworkPtr network) { }
        void solve() { }
        void operator()() {
            solve();
        }
    };


}

#endif // __BWPATHFINDER_HPP__