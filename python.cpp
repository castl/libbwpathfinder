#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
using namespace boost::python;

#include "bwpathfinder.hpp"

namespace bwpathfinder {

BOOST_PYTHON_MODULE(bwpathfinder) {
	class_<Link, LinkPtr>("Link", init<NodePtr, NodePtr, float>())
		.def_readonly("a", &Link::a)
		.def_readonly("b", &Link::b)
		.def_readwrite("bandwidth", &Link::bandwidth)
		.def_readwrite("used_bandwidth", &Link::used_bandwidth)
		.def_readwrite("maximum_paths", &Link::maximum_paths)
	;

	class_<Node, NodePtr>("Node", init<>())
		.def_readonly("id", &Node::id)
	;
	
	class_<Path, PathPtr>("Path")
		.def_readwrite("src", &Path::src)
		.def_readwrite("dst", &Path::dst)
		.def_readwrite("delivered_bw", &Path::delivered_bw)
		.def_readwrite("path", &Path::path)
	;

	class_<std::vector<LinkPtr> >("LinkVec")
        .def(vector_indexing_suite<std::vector<LinkPtr>, true >())
    ;

	class_<Network, NetworkPtr>("Network", init<>())
		.def("addLink", &Network::addLink)
		.def("addPath", &Network::addPath)
		.def("getPaths", &Network::getPaths)
		.def("findLink", &Network::findLink)
		.def("computeDeliveredBandwidth", &Network::computeDeliveredBandwidth)
	;

	class_<Pathfinder, PathfinderPtr>("Pathfinder", init<>())
		.def("init", &Pathfinder::init)
		.def("solve", &Pathfinder::solve)
	;

	class_<std::vector<PathPtr> >("PathVec")
        .def(vector_indexing_suite<std::vector<PathPtr>, true >())
    ;
}

};