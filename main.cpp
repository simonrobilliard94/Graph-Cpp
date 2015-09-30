#include <iostream>
#include <assert.h>
#include "Graph.hpp"


int main() {
    cs6771::Graph<std::string,int> gCopy;

    // create some data to store as nodes.
    std::string s = "a";
    std::string t = "b";
    std::string u = "c";

    // add this data into the graph
    gCopy.addNode(s);
    gCopy.addNode(t);
    gCopy.addNode(u);

    gCopy.addEdge(u,t,1);
    gCopy.addEdge(u,t,2);

    const auto& constGraph = gCopy;

    std::cout << std::boolalpha << constGraph.isNode("a") << std::endl;
    //std::cout << std::boolalpha << constGraph.isConnected("a","b") << std::endl;
    //std::cout << std::boolalpha << constGraph.isConnected("c","b") << std::endl;

    std::cout << "Const graph: " << std::endl;
    constGraph.printNodes();
    for (auto n : constGraph) {
        std::cout << n;
    }

    for (auto n = constGraph.begin(); n != constGraph.end(); ++n) {
        //constGraph.printEdges(*n);
    }

    /*for (auto e = constGraph.edgeIteratorBegin("c"); e != constGraph.edgeIteratorEnd(); ++e) {
        std::cout << (*e).first << " " << (*e).second << std::endl;
    }*/
}