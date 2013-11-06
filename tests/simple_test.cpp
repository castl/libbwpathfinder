#include "bwpathfinder.hpp"

#define XY 3

using namespace bwpathfinder;

int main(void) {
    std::vector< std::vector<NodePtr> > nodes;
    for (int x=0; x<XY; x++) {
        std::vector<NodePtr> row;
        for (int y=0; y<XY; y++) {
            char buf[100];
            snprintf(buf, 100, "(%d,%d)", y, x);
            NodePtr n(new Node(1e-9));
            n->label = buf;
            row.push_back(n);
        }
        nodes.push_back(row);
    }

    NetworkPtr net(new Network(5));

    for (int x=0; x<XY; x++) {
        for (int y=0; y<XY; y++) {
            if ((x+1) < XY)
                net->addLink(nodes[x][y], nodes[x+1][y], 10e9, 1e-9);
            if ((y+1) < XY)
                net->addLink(nodes[x][y], nodes[x][y+1], 10e9, 1e-9);
        }
    }

    PathPtr p(new Path());
    p->src = NodePtr(new Node(1e-9));
    p->src->label = "p1src";
    p->dst = NodePtr(new Node(1e-9));
    p->dst->label = "p1dst";
    p->requested_bw = 2.0e10;
    net->addLink(p->src, nodes[0][0], 10e9, 1e-9);
    net->addLink(p->dst, nodes[0][2], 10e9, 1e-9);
    p->path.push_back(net->findLink(p->src, nodes[0][0]));
    p->path.push_back(net->findLink(nodes[0][0], nodes[0][1]));
    p->path.push_back(net->findLink(nodes[0][1], nodes[0][2]));
    p->path.push_back(net->findLink(nodes[0][2], p->dst));

    PathPtr p2(new Path());
    p2->src = NodePtr(new Node(1e-9));
    p2->src->label = "p2src";
    p2->dst = NodePtr(new Node(1e-9));
    p2->dst->label = "p2dst";
    p2->requested_bw = 2.0e10;
    net->addLink(p2->src, nodes[0][1], 10e9, 1e-9);
    net->addLink(p2->dst, nodes[0][2], 10e9, 1e-9);
    p2->path.push_back(net->findLink(p2->src, nodes[0][1]));
    p2->path.push_back(net->findLink(nodes[0][1], nodes[0][2]));
    p2->path.push_back(net->findLink(nodes[0][2], p2->dst));

    net->addPath(p);
    net->addPath(p2);

    net->simulateDeliveredBandwidth();

    printf("XY Path 1 bw: %e\n", p->delivered_bw);
    printf("XY Path 2 bw: %e\n", p2->delivered_bw);


    Pathfinder pf;
    pf.init(net, 1.0e7);
    float cost = pf.solveToBwPcnt(99.0, 5);
    printf("PF cost: %e\n", cost);

    net->simulateDeliveredBandwidth();

    printf("PF Path 1 bw: %e\n", p->delivered_bw);
    printf("PF Path 2 bw: %e\n", p2->delivered_bw);
}