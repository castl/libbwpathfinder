#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
using namespace boost::python;

#include "bwpathfinder.hpp"

namespace bwpathfinder {

    void check_pyerror() {
        if(PyErr_CheckSignals() == -1) {
            exit(1);
        } 
    }

BOOST_PYTHON_MODULE(bwpathfinder) {

	class_<Link, LinkPtr, boost::noncopyable>("Link", no_init)
		.def_readonly("a", &Link::a)
		.def_readonly("b", &Link::b)
		.def_readwrite("bandwidth", &Link::bandwidth)
		.def_readwrite("maximum_paths", &Link::maximum_paths)
		.def_readonly("bwRequested", &Link::bwRequested)
		.def("getPaths", &Link::getPaths)
	;

	class_<Node, boost::noncopyable>("Node", init<float>())
		.def_readonly("id", &Node::id)
	;

	class_<Path, PathPtr, boost::noncopyable>("Path")
		.def_readwrite("src", &Path::src)
		.def_readwrite("dst", &Path::dst)
		.def_readwrite("delivered_bw", &Path::delivered_bw)
		.def_readwrite("requested_bw", &Path::requested_bw)
		.def_readwrite("path", &Path::path)
		.def("calcCost", &Path::calcCost)
	;

	class_<std::vector<LinkPtr> >("LinkVec")
        .def(vector_indexing_suite<std::vector<LinkPtr>, true >())
    ;

	class_<Network, NetworkPtr, boost::noncopyable>("Network", init<unsigned long>())
		.def("addLink", &Network::addLink)
		.def("addPath", &Network::addPath)
		.def("getPaths", &Network::getPaths)
		.def("findLink", &Network::findLink)
		.def("simulateDeliveredBandwidth", &Network::simulateDeliveredBandwidth)
	;

	class_<Pathfinder, PathfinderPtr, boost::noncopyable>("Pathfinder", init<>())
		.def("init", &Pathfinder::init)
		.def("solve", &Pathfinder::solve)
		.def("solveConverge", &Pathfinder::solveConverge)
		.def_readonly("iteration", &Pathfinder::iteration)
		.def_readonly("cost", &Pathfinder::cost)
		.def_readwrite("overageCostIncrement", &Pathfinder::overageCostIncrement)
		.def_readwrite("historyCostIncrement", &Pathfinder::historyCostIncrement)
	;

	class_<std::vector<PathPtr> >("PathVec")
        .def(vector_indexing_suite<std::vector<PathPtr>, true >())
    ;
}

};