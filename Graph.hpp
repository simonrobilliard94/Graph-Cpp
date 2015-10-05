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
#include <utility>

namespace cs6771 {

    template <typename Node, typename Edge>
    class Graph {
    public:
        Graph(): nodes{}, ordered_edges{} {}
        Graph(const Graph &g): nodes{}, ordered_edges{} {
            for(auto it = g.nodes.cbegin(); it != g.nodes.cend(); ++it) {
                Node n{it->getNode()};
                NodeContainer nc{n};
                nodes.push_back(nc);
            }

            for(auto it = nodes.begin(); it != nodes.end(); ++it) {
                for(auto et = g.edgeIteratorBegin(it->getNode()); et != g.edgeIteratorEnd(); ++et) {
                    addEdge(it->getNode(), et->first, et->second);
                }
            }
        }

        Graph& operator=(const Graph& g) {
            this->clear();
            for(auto it = g.nodes.cbegin(); it != g.nodes.cend(); ++it) {
                Node n{it->getNode()};
                NodeContainer nc{n};
                this->nodes.push_back(nc);
            }

            for(auto it = this->nodes.begin(); it != this->nodes.end(); ++it) {
                for(auto et = g.edgeIteratorBegin(it->getNode()); et != g.edgeIteratorEnd(); ++et) {
                    this->addEdge(it->getNode(), et->first, et->second);
                }
            }
            return *this;
        }


        bool addNode(const Node& node);
        bool addEdge(const Node& start, const Node& end, const Edge& weight);
        void deleteNode(const Node& node) noexcept;
        void deleteEdge(const Node& start, const Node& end, const Edge& weight);
        bool replace(const Node& target, const Node& replacement);
        bool isNode(const Node& node) const;
        void printNodes() const;
        void clear() noexcept;
        bool isConnected(const Node& start, const Node& end) const;
        void mergeReplace(const Node& destroy, const Node& merge);
        void printEdges(const Node& n) const;

    private:
        class NodeContainer {
        public:
            NodeContainer(const Node& node);

            const Node& getNode() const {
                return *nodePtr;
            };
            Node& getNode() { return *nodePtr; }
            bool addEdge(const NodeContainer& destination, const Edge& weight);
            std::shared_ptr<Node> getNodePtr() const;
            void setNode(const Node& replacement);

            bool operator<(const NodeContainer &other) const {
                if(edges.size() != other.edges.size()) {
                    return edges.size() > other.edges.size();
                }
                return *nodePtr < *other.nodePtr;
            }

            bool hasEdge(const Node end, const Edge weight) const;
            bool isConnected(const Node end) const;

            void removeEdge(const Node& end, const Edge& weight) noexcept;
            void merge(NodeContainer destroy);

            void printEdges() const;

            std::vector<std::pair<Node, Edge>> generateOrder() const;

        private:
            class EdgeContainer {
            public:
                EdgeContainer(const NodeContainer& destination, const Edge& weight);
                const Edge& getWeight() const;
                const Node& getDestination() const;
                bool isValid() const;

            private:
                Edge weight_;
                std::weak_ptr<Node> destination_;
            };

            std::vector<EdgeContainer> edges;
            std::shared_ptr<Node> nodePtr;
        };
        // mutable because we need to re-order nodes upon iteration
        mutable std::vector<NodeContainer> nodes;
        mutable std::vector<std::pair<Node, Edge>> ordered_edges;


    public:
        class Iterator {
        public:
            typedef std::ptrdiff_t difference_type;
            typedef std::forward_iterator_tag iterator_category;
            typedef typename std::remove_const<Node>::type value_type;
            typedef const Node* pointer;
            typedef const Node& reference;

            typename Iterator::reference operator*() const;
            typename Iterator::pointer operator->() const;
            Iterator& operator++();

            bool operator==(const Iterator& other) const;
            bool operator!=(const Iterator& other) const;

            Iterator(typename std::vector<NodeContainer>::const_iterator it);

        private:
            typename std::vector<NodeContainer>::const_iterator it_;
        };

        Iterator begin() const;
        Iterator end() const;

        typename std::vector<std::pair<Node, Edge>>::const_iterator edgeIteratorBegin(const Node& node) const;
        typename std::vector<std::pair<Node, Edge>>::const_iterator edgeIteratorEnd() const;
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
            throw std::runtime_error("Expected nodes to exist");
        }

        return startNC->addEdge(*endNC, weight);
    };

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::isNode(const Node& node) const {
        auto nc = std::find_if(nodes.cbegin(), nodes.cend(), [&node] (const NodeContainer& nodeContainer) {
            const Node n1 = nodeContainer.getNode();
            return n1 == node;
        });

        return nc != nodes.cend();
    }

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::replace(const Node &target, const Node &replacement) {
        if(!isNode(target)) {
            throw std::runtime_error("Expected node to exist");
        }
        if(isNode(replacement)) {
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
    void Graph<Node, Edge>::deleteNode(const Node &node) noexcept {
        auto target = std::find_if(nodes.begin(), nodes.end(),
                                   [&node] (const NodeContainer nc) {
                                       return nc.getNode() == node;
                                   });
        if(target != nodes.end()) {
            nodes.erase(target);
        }
    }

    template <typename Node, typename Edge>
    typename Graph<Node,Edge>::Iterator Graph<Node,Edge>::begin() const {
        std::sort(nodes.begin(), nodes.end(), [] (const NodeContainer a, const NodeContainer b) {
            return a < b;
        });
        return Iterator{nodes.cbegin()};
    }

    template <typename Node, typename Edge>
    typename Graph<Node,Edge>::Iterator Graph<Node,Edge>::end() const {
        return Iterator{nodes.cend()};
    }

    template <typename Node, typename Edge>
    void Graph<Node, Edge>::printNodes() const {
        for(auto node: *this) {
            std::cout << node << std::endl;
        }
    }

    template <typename Node, typename Edge>
    void Graph<Node, Edge>::clear() noexcept {
        std::vector<NodeContainer> newcontainer{};
        nodes = newcontainer;
    }

    template <typename Node, typename Edge>
    void Graph<Node, Edge>::deleteEdge(const Node &start, const Node &end, const Edge &weight) {
        auto target = std::find_if(nodes.begin(), nodes.end(), [&start]
                (const NodeContainer& nc) {
            return nc.getNode() == start;
        });
        if(target == nodes.end()) {
            return;
        }

        target->removeEdge(end, weight);
    }

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::isConnected(const Node &start, const Node &end) const {
        if(!isNode(start) || !isNode(end)) {
            throw std::runtime_error("start or end node did not exist");
        }

        auto nc = std::find_if(nodes.begin(), nodes.end(), [&start] (const NodeContainer nc) {
            return nc.getNode() == start;
        });

        if(nc != nodes.end()) {
            return nc->isConnected(end);
        }
        return false;
    }

    template <typename Node, typename Edge>
    void Graph<Node, Edge>::mergeReplace(const Node &destroy, const Node &merge) {
        if(!isNode(destroy) || !isNode(merge)) {
            throw std::runtime_error("Expected the nodes to be defined");
        }
        auto dnc = std::find_if(nodes.begin(), nodes.end(), [&destroy] (const NodeContainer& nc) {
            return nc.getNode() == destroy;
        });

        auto mnc = std::find_if(nodes.begin(), nodes.end(), [&merge] (const NodeContainer& nc) {
            return nc.getNode() == merge;
        });

        mnc->merge(*dnc);

        nodes.erase(dnc);
    }

    template <typename Node, typename Edge>
    void Graph<Node, Edge>::printEdges(const Node &n) const {
        if(!isNode(n)) {
            throw std::runtime_error("Could not find node");
        }

        auto nc = std::find_if(nodes.cbegin(), nodes.cend(), [&n] (const NodeContainer nc) {
            return nc.getNode() == n;
        });

        if(nc != nodes.cend()) {
            nc->printEdges();
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
            if(it->isValid() && it->getDestination() == destination.getNode() && it->getWeight() == weight) {
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
    std::shared_ptr<Node> Graph<Node, Edge>::NodeContainer::getNodePtr() const {
        return nodePtr;
    }

    template <typename Node, typename Edge>
    void Graph<Node, Edge>::NodeContainer::setNode(const Node &replacement) {
        *nodePtr = replacement;
    }

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::NodeContainer::hasEdge(const Node end, const Edge weight) const {
        auto ec = std::find_if(edges.cbegin(), edges.cend(), [&end, &weight] (const EdgeContainer& ec) {
            return (ec.isValid() && ec.getDestination() == end && ec.getWeight() == weight);
        });

        return ec != edges.cend();
    };

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::NodeContainer::isConnected(const Node end) const {
        auto ec = std::find_if(edges.cbegin(), edges.cend(), [&end] (const EdgeContainer& ec) {
            return (ec.isValid() && ec.getDestination() == end);
        });

        return ec != edges.cend();
    };

    template <typename Node, typename Edge>
    void Graph<Node, Edge>::NodeContainer::removeEdge(const Node &end, const Edge &weight) noexcept {
        auto toRemove = std::find_if(edges.begin(), edges.end(), [&end, &weight] (const EdgeContainer ec) {
            return (ec.isValid() && ec.getDestination() == end && ec.getWeight() == weight);
        });

        if(toRemove != edges.end()) {
            edges.erase(toRemove);
        }
    }

    template <typename Node, typename Edge>
    void Graph<Node, Edge>::NodeContainer::merge(NodeContainer destroy) {
        while(!destroy.edges.empty()) {
            EdgeContainer ec = destroy.edges.back();
            destroy.edges.pop_back();
            if(ec.isValid() && ec.getDestination() != *nodePtr && !hasEdge(ec.getDestination(), ec.getWeight())) {
                edges.push_back(ec);
            }
        }
    }

    template <typename Node, typename Edge>
    void Graph<Node, Edge>::NodeContainer::printEdges() const {
        std::vector<std::pair<Node, Edge>> order = generateOrder();
        std::cout << "Edges attached to Node " << getNode() << std::endl;
        for(auto e = order.cbegin(); e != order.cend(); ++e) {
            std::cout << e->first << " " << e->second << std::endl;
        }
        if(order.size() == 0) {
            std::cout << "(null)" << std::endl;
        }
    }

    template <typename Node, typename Edge>
    std::vector<std::pair<Node, Edge>> Graph<Node, Edge>::NodeContainer::generateOrder() const {
        std::vector<std::pair<Node, Edge>> newOrder{};
        for(auto it = edges.cbegin(); it != edges.end(); ++it) {
            if(it->isValid()) {
                std::pair<Node, Edge>  p= std::make_pair(it->getDestination(), it->getWeight());
                newOrder.push_back(p);
            }
        }
        std::sort(newOrder.begin(), newOrder.end(), []
                (const std::pair<Node, Edge> a, const std::pair<Node, Edge> b) {
            if(a.second == b.second) {
                return a.first < b.first;
            }
            return a.second < b.second;
        });
        return newOrder;
    }

    /**************** Edge Container Methods *********************/

    template <typename Node, typename Edge>
    Graph<Node, Edge>::NodeContainer::EdgeContainer::EdgeContainer(const NodeContainer &destination, const Edge &weight):
            weight_{weight}, destination_{destination.getNodePtr()} {

    }

    template <typename Node, typename Edge>
    const Edge& Graph<Node, Edge>::NodeContainer::EdgeContainer::getWeight() const {
        return weight_;
    }

    template <typename Node, typename Edge>
    const Node& Graph<Node, Edge>::NodeContainer::EdgeContainer::getDestination() const {
        return *destination_.lock();
    }

    template <typename Node, typename Edge>
    bool Graph<Node, Edge>::NodeContainer::EdgeContainer::isValid() const {
        if(destination_.lock() != nullptr) {
            return true;
        }
        return false;
    }

    /******************* Iterator Methods ****************************/

    template <typename Node, typename Edge>
    Graph<Node, Edge>::Iterator::Iterator(typename std::vector<NodeContainer>::const_iterator it): it_{it} {

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

    /**************** edge iterator methods **********************/
    template <typename Node, typename Edge>
    typename std::vector<std::pair<Node, Edge>>::const_iterator Graph<Node, Edge>::edgeIteratorBegin(const Node &node) const {
        if(!isNode(node)) {
            throw std::runtime_error("could not find node in graph");
        }
        auto nc = std::find_if(nodes.begin(), nodes.end(), [&node] (const NodeContainer nc) {
            return nc.getNode() == node;
        });

        this->ordered_edges = nc->generateOrder();
        return this->ordered_edges.cbegin();
    }

    template <typename Node, typename Edge>
    typename std::vector<std::pair<Node, Edge>>::const_iterator Graph<Node, Edge>::edgeIteratorEnd() const {
        return ordered_edges.cend();
    }

};



#endif //GRAPH_GRAPH_HPP
