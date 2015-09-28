//
// Created by Simon Robilliard on 23/09/15.
//

#ifndef GRAPH_GRAPH_HPP
#define GRAPH_GRAPH_HPP

#include <vector>
#include <iterator>
#include <cassert>

namespace cs6771 {

    //template <typename Node, typename Edge> class Graph;

    template <typename Node, typename Edge>
    class Graph {
    public:
        /**
         * Default Constructor
         */
        //friend class Edge_Iterator<Edge>;
        //typedef Graph_Iterator<Edge> iterator;
        //typedef Edge_Iterator edgeIterator;
        bool addNode(const Node& node);
        bool addEdge(const Node& start, const Node& end, const Edge& weight);
        bool deleteNode(const Node& node);
        //bool deleteEdge(const Node& start, const Node& end, const Edge& weight);
        bool replace(const Node& target, const Node& replacement);
        //void mergeReplace(const Node& source, const Node& target);*/
        bool isNode(const Node& node);
        //iterator begin() {return iterator{nodes, 0};}

        const Edge_Iterator& edgeIteratorBegin() const;
        const Edge_Iterator& edgeIteratorEnd() const;


        class Edge_Iterator {
        public:
            typedef std::ptrdiff_t                     difference_type;
            typedef std::forward_iterator_tag          iterator_category;
            typedef Edge                                  value_type;
            typedef Edge*                                 pointer;
            typedef Edge&                                 reference;

            reference operator*() const;
            pointer operator->() const { return &(operator*()); }
            Edge_Iterator& operator++();
            bool operator==(const Edge_Iterator& other) const;
            bool operator!=(const Edge_Iterator& other) const { return !operator==(other); }

            Edge_Iterator(const std::vector<Edge> collection, unsigned int position):
                    collection_{collection}, position_{position} {}

        private:
            const std::vector<Edge> collection_;
            unsigned int position_;
        };


    private:
        class NodeContainer {
        public:
            NodeContainer(const Node& node);

            const Node& getNode() const;
            bool addEdge(const NodeContainer& destination, const Edge& weight);
            std::shared_ptr<Node> getNodePtr() const;
            void setNode(const Node& replacement);

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
        if(target != nodes.end) {
            nodes.erase(target);
        }
    }

    template <typename Node, typename Edge>


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

    /************ Edge Iterator Methods ****************************/
    /*template <typename Node, typename Edge>
    reference Graph<Node, Edge>::Edge_Iterator::operator*() const {
        return collection_.at(position_);
    }*/

    template <typename Node, typename Edge>
    typename Graph<Node, Edge>::Edge_Iterator::reference Graph<Node, Edge>::Edge_Iterator::operator*() const {

    }


    template <typename Node, typename Edge>
    typename Graph<Node, Edge>::Edge_Iterator& Graph<Node, Edge>::Edge_Iterator::operator++() {
        if(position_ < collection_.size()) {
            position_++;
        }
        return *this;
    }

    template<typename Node, typename Edge>
    bool Graph<Node, Edge>::Edge_Iterator::operator==(const Edge_Iterator &other) const {
        return collection_[position_] == other.collection_[other.position_];
    }

};



#endif //GRAPH_GRAPH_HPP
