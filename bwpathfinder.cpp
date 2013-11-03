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
        	path->assign(path->path, path->delivered_bw);
        }
    }

    float Link::costToUse(float hopCost, float bw) const {
    	float totBW = bwRequested + bw;
        float overage = totBW > bandwidth ? totBW - bandwidth : 0;
        return hopCost + pow(overage, overageExponent);
    }

    float Pathfinder::solveConverge(float improvementThreshold, uint64_t maxIter) {
    	std::deque<float> costs;
    	printf("Pathfinder solveConverge:\n");
    	printf("\tNodes: %lu\n\tLinks: %lu\n\tPaths: %lu\n\tHopCost:%e\n\tThresh: %e\n",
    			network->nodes.size(), network->links.size(), network->paths.size(),
    			hopCost, improvementThreshold);
    	while (iteration < maxIter) {
    		float icost = iterate();
    		cost = solutionCost();
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

    		printf("\tIteration %lu cost: %e  linkcost: %e\n", iteration, icost, cost);

    		float diff = max - min;
    		if (costs.size() >= 3 && diff < improvementThreshold) {
    			break;
    		}
    		if (cost < 0.1) {
    			// < 1 is a very small cost
    			break;
    		}
    	}

    	return cost;
    }

    float Pathfinder::solve(float desiredCost, uint64_t maxIter) {
    	while (cost > desiredCost && iteration < maxIter) {
    		iterate();
    		cost = solutionCost();
    		iteration += 1;
    	}

    	return cost;
    }

    struct PartialPath {
        PathPtr defn;
        float cost;
        float bw;
        NodePtr node;
        std::vector<LinkPtr> path;

        PartialPath(PathPtr defn) :
            defn(defn),
            cost(0),
            bw(defn->requested_bw),
            node(defn->src) {
        }

        PartialPath(float hopCost, const PartialPath& orig, LinkPtr next) :
            defn(orig.defn),
            cost(orig.cost),
            bw(next->bwShareW(orig.bw)),
            node(orig.node),
            path(orig.path) {
            path.push_back(next);
            float nextCost = next->costToUse(hopCost, bw);
            cost = cost + nextCost;

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
            	printf("\t%s -- %s : bw %e, req_bw: %e, paths: %lu\n",
            		link->a->label.c_str(), link->b->label.c_str(), link->bandwidth,
            		link->bwRequested, link->paths.size());
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
    	std::vector<PathPtr> allPaths;

    	if (iteration == 0) {
	    	allPaths = this->network->getPaths();
	    } else {
	    	std::set<PathPtr, smart_ptr_less_than> congestedPaths;
	    	for (LinkPtr link : this->network->links) {
	    		if (link->bwRequested > link->bandwidth) {
	    			congestedPaths.insert(link->paths.begin(), link->paths.end());
	    		}
	    	}
	    	allPaths = std::vector<PathPtr>(congestedPaths.begin(), congestedPaths.end());
	    }
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
    				path->assign(bestPath.path, bestPath.bw);
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
    		link->incrementPenalties(0.01, 0.1);
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
