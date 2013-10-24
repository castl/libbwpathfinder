#ifndef __BWPATHFINDER_HPP__
#define __BWPATHFINDER_HPP__

#include <vector>
#include <set>
#include <boost/shared_ptr.hpp>

namespace bwpathfinder {

    struct Node;
    typedef boost::shared_ptr<Node> NodePtr;
    // typedef boost::shared_ptr<const Node> NodeCPtr;

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

    typedef boost::shared_ptr<Link> LinkPtr;
    // typedef boost::shared_ptr<const Link> LinkCPtr;

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
        float delivered_bw;
        std::vector<LinkPtr> path;

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

    public:
        Network() { }

        void add_link(NodePtr src, NodePtr dst, float bw) {

        }
    };

    typedef boost::shared_ptr<Network> NetworkPtr;

    class TrafficPattern {
        struct Traffic {
            NodePtr src;
            NodePtr dst;
            float       bandwidth;
        };

        std::vector<Traffic> all_traffic;
    public:
        TrafficPattern() { }

        void add_traffic(NodePtr src, NodePtr dst, float bandwidth) { }
    };

    typedef boost::shared_ptr<TrafficPattern> TrafficPatternPtr;

    class Pathfinder {
        std::vector<Path> paths;

        void iterate0();
        void iterate();
    public:
        Pathfinder() { }
        Pathfinder(NetworkPtr network, TrafficPatternPtr traffic) {
            this->init(network, traffic);
        }

        void init(NetworkPtr network, TrafficPatternPtr traffic) { }
        void solve() { }
        void operator()() {
            solve();
        }

        std::vector<Path> getSolutionPaths() {
            return paths;
        }
    };

    typedef boost::shared_ptr<Pathfinder> PathfinderPtr;

}

#endif // __BWPATHFINDER_HPP__