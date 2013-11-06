#include "bwpathfinder.hpp"
#include "network_simulation.hpp"

#include <queue>
#include <unordered_set>
#include <cstdio>

namespace bwpathfinder {

    size_t Node::id_counter = 0;

    void Network::simulateDeliveredBandwidth() {
        NetworkSimulation sim(shared_from_this());
        simfw::Timer<Time> checkup(&sim, 1e-8, [] (uint64_t i) {
            check_pyerror();
            return true;
        });
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
    	// float totBW = bwRequested + bw;
        // float overage = totBW > bandwidth ? totBW - bandwidth : 0;
        float overage = std::max((float)0.0, bw - this->bwShareW(bw));
        return hopCost + (overage * overageExponent) + historyPenalty;
    }

    float Pathfinder::solveToBwPcnt(float desiredPcnt, uint64_t maxIter) {
        desiredPcnt = std::min((float)100.0, desiredPcnt);
        float requested_bw = 0.0;
        for (PathPtr path: this->network->paths) {
            requested_bw += path->requested_bw;
        }

    	printf("Pathfinder solveToBwPcnt:\n");
    	printf("\tNodes: %lu\n\tLinks: %lu\n\tPaths: %lu\n\tHopCost:%e\n\tGoal: %f%%\n\tBandwidth: %e\n",
    			network->nodes.size(), network->links.size(), network->paths.size(),
    			hopCost, desiredPcnt, requested_bw);
    	while (iteration < maxIter) {
    		float icost = iterate();
    		cost = solutionCost();
            float bw = deliveredBw();
            float pcnt = 100 * bw / requested_bw;
    		iteration += 1;

    		printf("\tIteration %lu cost: %e  linkcost: %e  bw: %e (%f%%)\n",
                    iteration, icost, cost, bw, 100.0 * bw / requested_bw);

            if (pcnt >= desiredPcnt)
                break;
    	}

    	return cost;
    }

    float Pathfinder::solveConverge(float improvementThreshold, uint64_t maxIter) {
        std::deque<float> costs;
        float requested_bw = 0.0;
        for (PathPtr path: this->network->paths) {
            requested_bw += path->requested_bw;
        }
        printf("Pathfinder solveConverge:\n");
        printf("\tNodes: %lu\n\tLinks: %lu\n\tPaths: %lu\n\tHopCost:%e\n\tThresh: %e\n\tBandwidth: %e\n",
                network->nodes.size(), network->links.size(), network->paths.size(),
                hopCost, improvementThreshold, requested_bw);
        while (iteration < maxIter) {
            float icost = iterate();
            cost = solutionCost();
            float bw = deliveredBw();
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

            printf("\tIteration %lu cost: %e  linkcost: %e  bw: %e (%f%%)\n",
                    iteration, icost, cost, bw, 100.0 * bw / requested_bw);

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
        Path* defn;
        float cost;
        float bw;
        Node* node;
        std::vector<Link*> path;

        PartialPath(const PathPtr& defn) :
            defn(defn.get()),
            cost(0),
            bw(defn->requested_bw),
            node(defn->src.get()) {
        }

        PartialPath(float hopCost, const PartialPath& orig, Link* next) :
            defn(orig.defn),
            cost(orig.cost),
            bw(next->bwShareW(orig.bw)),
            node(orig.nextOverLink(next)),
            path(orig.path) {
            path.push_back(next);
            float nextCost = next->costToUse(hopCost, bw);
            cost = cost + nextCost;
        }

        Node* nextOverLink(Link* next) const {
            if (node == next->a.get())
                return next->b.get();
            else if (node == next->b.get())
                return next->a.get();
            else
                assert(false && "Link being added doesn't connect");
        }

        void print() {
        	const char* fullPart = sinkFound() ? "Full" : "Part";
            printf("%s path  cost: %e node: %p\n", fullPart, cost, node);
            for (Link* link : path) {
            	printf("\t%s -- %s : bw %e, req_bw: %e, paths: %lu\n",
            		link->a->label.c_str(), link->b->label.c_str(), link->bandwidth,
            		link->bwRequested, link->paths.size());
            }
        }

        Link* last() {
            if (path.size() == 0)
                return NULL;
            return path.back();
        }

        bool sinkFound() {
            if (path.size() == 0)
                return defn->src == defn->dst;
            Link* lst = path.back();
            return lst->a == defn->dst || lst->b == defn->dst;
        }

        bool operator<(const PartialPath& other) const {
            return this->cost > other.cost;
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
            check_pyerror();

    		path->ripup();

    		// Find a new path
            std::unordered_set<Node*> nodesSeen;
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
    			} else if (nodesSeen.find(bestPath.node) == nodesSeen.end()) {
                    nodesSeen.insert(bestPath.node);
    				// printf("\tAt node: %s\n", node->label.c_str());
    				auto& links = this->network->linkIndex[bestPath.node];
    				for (Link* link : links) {
    					// printf("\tConsidering: %s -- %s\n",
    						// link->a->label.c_str(), link->b->label.c_str());
						// Don't include the link we just used
    					if (link != bestPath.last()) {
                            Node* next = bestPath.nextOverLink(link);
                            if (nodesSeen.find(next) == nodesSeen.end()) {
                                PartialPath pp(this->hopCost, bestPath, link);
                                q.push(pp);
                            }
    					}
    				}
    			}
    		}
    	}

    	// Increment history penalties
    	for (LinkPtr link : this->network->links) {
    		link->incrementPenalties(this->historyCostIncrement, this->overageCostIncrement);
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

    float Pathfinder::deliveredBw() {
        float bw = 0.0;
        for (PathPtr path: this->network->paths) {
            bw += path->delivered_bw;
        }
        return bw;
    }
}
