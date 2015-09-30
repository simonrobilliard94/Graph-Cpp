//
// Created by Simon Robilliard on 23/09/15.
//

#ifndef GRAPH_GRAPH_HPP
#define GRAPH_GRAPH_HPP

#include <vector>
#include <iterator>
#include <cassert>
#include <memory>
#include <algorithm>
#include <iostream>

namespace cs6771 {

    template <typename Node, typename Edge>
    class Graph {
    public:

        bool addNode(const Node& node);
        bool addEdge(const Node& start, const Node& end, const Edge& weight);
        bool deleteNode(const Node& node);
        bool replace(const Node& target, const Node& replacement);
        bool isNode(const Node& node);
        void printNodes();

    private:
        class NodeContainer {
        public:
            NodeContainer(const Node& node);

            const Node& getNode() const;
            Node& getNode() { return *nodePtr; }
            bool addEdge(const NodeContainer& destination, const Edge& weight);
            std::shared_ptr<Node> getNodePtr() const;
            void setNode(const Node& replacement);
            std::vector<Edge> getEdges() const;

            bool operator<(const NodeContainer &other) const {
                if(edges.size() != other.edges.size()) {
                    return edges.size() > other.edges.size();
                }
                return *nodePtr < *other.nodePtr;
            }

        private:
            class EdgeContainer {
            public:
                EdgeContainer(const NodeContainer& destination, const Edge& weight);
                const Edge& getWeight() const;
                const Node& getDestination() const;
            private:
                Edge weight_;
                std::weak_ptr<Node> destination_;
            };

            std::vector<EdgeContainer> edges;
            std::shared_ptr<Node> nodePtr;
        };
        std::vector<NodeContainer> nodes;

    public:
        class Iterator {
        public:
            typedef std::ptrdiff_t difference_type;
            typedef std::forward_iterator_tag iterator_category;
            typedef Node value_type;
            typedef Node* pointer;
            typedef Node& reference;

            typename Iterator::reference operator*() const;
            typename Iterator::pointer operator->() const;
            Iterator& operator++();

            bool operator==(const Iterator& other) const;
            bool operator!=(const Iterator& other) const;

            Iterator(typename std::vector<NodeContainer>::iterator it);

        private:
            typename std::vector<NodeContainer>::iterator it_;
        };

        Iterator begin();
        Iterator end();
    };

    /***************** Graph Methods *****************************/

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::addNode(const Node& node) {
        if(isNode(node)) {
            return false;
        }
        NodeContainer newNode{node};
        nodes.push_back(newNode);
        return true;
    }

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::addEdge(const Node& start, const Node& end, const Edge& weight) {

        auto startNC = std::find_if(nodes.begin(), nodes.end(),
                                    [&start] (const NodeContainer& nodeContainer) {
                                        return start == nodeContainer.getNode();
                                    });

        auto endNC = std::find_if(nodes.begin(), nodes.end(),
                                    [&end] (const NodeContainer& nodeContainer) {
                                        return end == nodeContainer.getNode();
                                    });

        if(startNC == nodes.end() || endNC == nodes.end()) {
            return false;
        }

        return startNC->addEdge(*endNC, weight);
    };

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::isNode(const Node& node) {
        auto nc = std::find_if(nodes.begin(), nodes.end(),
                               [&node] (const NodeContainer& nodeContainer) {
                                   return node == nodeContainer.getNode();
                               });
        return nc != nodes.end();
    }

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::replace(const Node &target, const Node &replacement) {
        if(!isNode(target)) {
            return false;
        }
        bool found = false;
        for(auto it = nodes.begin(); it != nodes.end() && !found; ++it) {
            if(it->getNode() == target) {
                it->setNode(replacement);
                found = true;
            }
        }
        return found;
    }

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::deleteNode(const Node &node) {
        auto target = std::find_if(nodes.begin(), nodes.end(),
                                   [&node] (const NodeContainer nc) {
                                       return nc.getNode() == node;
                                   });
        if(target != nodes.end()) {
            nodes.erase(target);
        }
    }

    template <typename Node, typename Edge>
    typename Graph<Node,Edge>::Iterator Graph<Node,Edge>::begin() {
        std::sort(nodes.begin(), nodes.end(), [] (const NodeContainer a, const NodeContainer b) {
            return a < b;
        });
        return Iterator{nodes.begin()};
    }

    template <typename Node, typename Edge>
    typename Graph<Node,Edge>::Iterator Graph<Node,Edge>::end() {
        return Iterator{nodes.end()};
    }

    template <typename Node, typename Edge>
    void Graph<Node, Edge>::printNodes() {
        for(auto node: *this) {
            std::cout << node << std::endl;
        }
    }

    /****************** Node Container Methods *********************/

    template <typename Node, typename Edge>
    Graph<Node, Edge>::NodeContainer::NodeContainer(const Node &node):
            nodePtr{std::make_shared<Node>(node)} {}

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::NodeContainer::addEdge(const NodeContainer &destination, const Edge &weight) {
        bool duplicate = false;
        for(auto it = edges.begin(); it != edges.end(); ++it) {
            if(it->getDestination() == destination.getNode() && it->getWeight() == weight) {
                duplicate = true;
            }
        }
        if(duplicate) {
            return false;
        }

        EdgeContainer newEdge{destination, weight};
        edges.push_back(newEdge);
        return true;
    }

    template <typename Node, typename Edge>
    const Node& Graph<Node, Edge>::NodeContainer::getNode() const {
        return *nodePtr;
    }

    template <typename Node, typename Edge>
    std::shared_ptr<Node> Graph<Node, Edge>::NodeContainer::getNodePtr() const {
        return nodePtr;
    }

    template <typename Node, typename Edge>
    void Graph<Node, Edge>::NodeContainer::setNode(const Node &replacement) {
        nodePtr = std::make_shared<Node>(replacement);
    }

    template <typename Node, typename Edge>
    std::vector<Edge> Graph<Node, Edge>::NodeContainer::getEdges() const {

        std::vector<Edge> rawEdges{};

        for(auto it = edges.cbegin(); it != edges.cend(); ++it) {
            rawEdges.push_back(it->getWeight());
        }

        return rawEdges;
    }

    /**************** Edge Container Methods *********************/

    template <typename Node, typename Edge>
    Graph<Node, Edge>::NodeContainer::EdgeContainer::EdgeContainer(const NodeContainer &destination, const Edge &weight):
            weight_{weight} {
        destination_ = destination.getNodePtr();
    }

    template <typename Node, typename Edge>
    const Edge& Graph<Node, Edge>::NodeContainer::EdgeContainer::getWeight() const {
        return weight_;
    }

    template <typename Node, typename Edge>
    const Node& Graph<Node, Edge>::NodeContainer::EdgeContainer::getDestination() const {
        return *destination_.lock();
    }

    /******************* Iterator Methods ****************************/

    template <typename Node, typename Edge>
    Graph<Node, Edge>::Iterator::Iterator(typename std::vector<NodeContainer>::iterator it): it_{it} {

    }

    template <typename Node, typename Edge>
    typename Graph<Node, Edge>::Iterator::reference Graph<Node, Edge>::Iterator::operator*() const {
        return it_->getNode();
    }


    template <typename Node, typename Edge>
    typename Graph<Node, Edge>::Iterator::pointer Graph<Node, Edge>::Iterator::operator->() const {
        return &(operator*());
    }

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::Iterator::operator==(const Iterator &other) const {
        return it_ == other.it_;
    }

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::Iterator::operator!=(const Iterator &other) const {
        return !(operator==(other));
    }

    template <typename Node, typename Edge>
    typename Graph<Node, Edge>::Iterator& Graph<Node, Edge>::Iterator::operator++() {
        ++it_;
        return *this;
    }

};



#endif //GRAPH_GRAPH_HPP
