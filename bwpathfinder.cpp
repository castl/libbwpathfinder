#include "bwpathfinder.hpp"
#include "network_simulation.hpp"

#include <queue>
#include <cstdio>

namespace bwpathfinder {

    size_t Node::id_counter = 0;

    void Network::simulateDeliveredBandwidth() {
        NetworkSimulation sim(shared_from_this());
        sim.simulate();
        sim.setDeliveredBandwidths();
        sim.flushMessages();

        // Zero all state variables
        for (LinkPtr link : this->links) {
            link->paths.clear();
            link->bwRequested = 0.0;
        }

        for (PathPtr path : this->paths) {
        	path->assign(path->path);
        }
    }

    float Link::costToUse(PathPtr path) const {
        float cost = path->requested_bw - bwShareW(path->requested_bw);
        return cost + historyPenalty;
    }

    float Pathfinder::solveConverge(float improvementThreshold, uint64_t maxIter) {
    	std::deque<float> costs;
    	while (iteration < maxIter) {
    		cost = iterate();
    		costs.push_back(cost);
    		while (costs.size() > 3) {
    			costs.pop_front();
    		}
    		iteration += 1;

    		float min = costs.front();
    		float max = min;
    		for (float c : costs) {
    			min = std::min(min, c);
    			max = std::max(max, c);
    		}

    		float diff = max - min;
    		if (costs.size() >= 3 && diff < improvementThreshold) {
    			break;
    		}
    	}

    	return cost;
    }

    float Pathfinder::solve(float desiredCost, uint64_t maxIter) {
    	while (cost > desiredCost && iteration < maxIter) {
    		cost = iterate();
    		iteration += 1;
    	}

    	return cost;
    }

    struct PartialPath {
        PathPtr defn;
        float cost;
        NodePtr node;
        std::vector<LinkPtr> path;

        PartialPath(PathPtr defn) :
            defn(defn),
            cost(0),
            node(defn->src) {
        }

        PartialPath(float hopCost, const PartialPath& orig, LinkPtr next) :
            defn(orig.defn),
            cost(orig.cost),
            node(orig.node),
            path(orig.path) {
            path.push_back(next);
            float nextCost = next->costToUse(defn) + (hopCost * path.size());
            cost = std::max(cost, nextCost);

            if (node == next->a)
                node = next->b;
            else if (node == next->b)
                node = next->a;
            else
                assert(false && "Link being added doesn't connect");
        }

        void print() {
        	const char* fullPart = sinkFound() ? "Full" : "Part";
            printf("%s path  cost: %e node: %p\n", fullPart, cost, node.get());
            for (LinkPtr link : path) {
            	printf("\t%s -- %s : bw %e, req_bw: %e, cost %e, paths: %lu\n",
            		link->a->label.c_str(), link->b->label.c_str(), link->bandwidth,
            		link->bwRequested, link->costToUse(defn), link->paths.size());
            }
        }

        LinkPtr last() {
            if (path.size() == 0)
                return LinkPtr();
            return path.back();
        }

        bool sinkFound() {
            if (path.size() == 0)
                return defn->src == defn->dst;
            LinkPtr lst = path.back();
            return lst->a == defn->dst || lst->b == defn->dst;
        }

        bool operator<(const PartialPath& other) const {
            return this->cost > other.cost;
        }

        bool hasCycle() const {
        	std::set<LinkPtr, smart_ptr_less_than> pathSet(path.begin(), path.end());
        	assert(pathSet.size() <= path.size());
        	return pathSet.size() < path.size();
        }
    };

    float Pathfinder::iterate() {
    	std::vector<PathPtr> allPaths = this->network->getPaths();
    	std::random_shuffle(allPaths.begin(), allPaths.end());

    	float cost = 0.0;
    	for (PathPtr path : allPaths) {
    		path->ripup();

    		// Find a new path
    		std::priority_queue<PartialPath> q;
    		q.push(PartialPath(path));

    		bool sinkFound = false;
    		while (!sinkFound) {
    			assert(!q.empty() && "Could not find path!");

    			PartialPath bestPath = q.top();
    			q.pop();
    			// bestPath.print();

    			if (bestPath.sinkFound()) {
    				// The current best path has found its destination
    				sinkFound = true;
    				path->assign(bestPath.path);
    				cost += bestPath.cost;
    				// printf("\t=== Assigned! === \n");
    			} else {
    				NodePtr node = bestPath.node;
    				// printf("\tAt node: %s\n", node->label.c_str());
    				auto& links = this->network->linkIndex[node];
    				for (LinkPtr link : links) {
    					// printf("\tConsidering: %s -- %s\n",
    						// link->a->label.c_str(), link->b->label.c_str());
						// Don't include the link we just used
    					if (link != bestPath.last()) {
    						PartialPath pp(this->hopCost, bestPath, link);
    						if (!pp.hasCycle()) {
	    						q.push(pp);
	    						// printf("\t\tpushed with cost %e\n", pp.cost);
	    					}
    					}
    				}
    			}
    		}
    	}

    	// Increment history penalties
    	for (LinkPtr link : this->network->links) {
    		link->recomputeHistoryPenalty(0.05);
    	}

    	return cost;
    }

    float Pathfinder::solutionCost() {
    	float cost = 0.0;
    	for (LinkPtr link : this->network->links) {
    		cost += link->solutionPartialCost();
    	}
    	return cost;
    }
}
