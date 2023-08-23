#ifndef RRTGRAPH_CPP
#define RRTGRAPH_CPP

#include "RRTGraph.h"
#include <sstream>
#include <cmath>
#include <iostream>

using namespace std;

template<class State>
RRTGraph<State>::RRTGraph() {
    nodes = (Node<State>*)malloc(RRTGRAPH_MAX_NODE_COUNT * sizeof(Node<State>));
}

template<class State>
RRTGraph<State>::~RRTGraph() {
    delete nodes;
}

template<class State>
Node<State> *RRTGraph<State>::addNode(State *_state, Node<State> *_parent, float _cost) {
    Node<State>* output = addNode(_state);
    output->parent = _parent;
    output->cost = _cost;
    return output;
}

template <class State>
Node<State>* RRTGraph<State>::addNode(State* _state) {
    if (nodes_size == RRTGRAPH_MAX_NODE_COUNT) {
        std::cerr << "Out of graph storage space!  Increase RRTGRAPH_MAX_NODE_COUNT." << std::endl;
        exit(1);
    }

    nodes[nodes_next_index].state = *_state;
    nodes[nodes_next_index].cost = NAN;
    nodes[nodes_next_index].parent = nullptr;
    nodes[nodes_next_index].next = nullptr;
    nodes[nodes_next_index].prev = node_last;
    if (node_first == nullptr) {
        node_first = &nodes[nodes_next_index];
    }
    if (node_last != nullptr) {
        node_last->next = &nodes[nodes_next_index];
    }
    node_last = &nodes[nodes_next_index];
    nodes_size++;
    return &nodes[nodes_next_index++];
}

template <class State>
void RRTGraph<State>::delNode(Node<State>* node) {
    Node<State>* node_i = first();
    while (node_i != nullptr) {
        if (node_i->parent == node) {
            delNode(node_i);
        }
        node_i = node_i->next;
    }
    if (node->next != nullptr) {
        node->next->prev = node->prev;
    }
    if (node->prev != nullptr) {
        node->prev->next = node->next;
    }
    if (node == node_first) {
        node_first = node->next;
    }
    nodes_size--;
}

template<class State>
Node<State> *RRTGraph<State>::atIndex(int _index) {
    return &nodes[_index];
}

template <class State>
Node<State>* RRTGraph<State>::find(State* _state) {
    Node<State>* node = node_first;
    while (node != nullptr) {
        if (node->state == *_state) {
            return node;
        }
        node = node->next;
    }
    return nullptr;
}

template<class State>
Node<State> *RRTGraph<State>::first() {
    return node_first;
}

template <class State>
int RRTGraph<State>::size() {
    return nodes_size;
}

template <class State>
string RRTGraph<State>::toString() {
    stringstream output;
    Node<State>* node = node_first;
    while (node != nullptr) {
        int i = node - &nodes[0];
        output << "Node " << i << " at " << node << ": " << nodes[i].state.toString()
               << " parent=" << node->parent
               << " cost=" << node->cost
               << endl;
        node = node->next;
    }
    return output.str();
}

#endif
