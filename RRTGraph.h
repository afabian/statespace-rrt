#ifndef RRTGRAPH_H
#define RRTGRAPH_H

#include <vector>
#include <map>
#include <string>

#define RRTGRAPH_MAX_NODE_COUNT 100001

template <class State>
class Node {
public:
    State state;
    float cost;
    Node* parent;
    Node* next;
    Node* prev;
};

template <class State>
class RRTGraph {

public:
    RRTGraph();
    ~RRTGraph();
    Node<State>* addNode(State* _state);
    void delNode(Node<State>*);
    Node<State>* atIndex(int _index);
    Node<State>* first();
    Node<State>* find(State* _state);
    int size();

    std::string toString();

private:
    Node<State>* nodes = nullptr;
    Node<State>* node_first = nullptr;
    Node<State>* node_last = nullptr;
    int nodes_next_index = 0;
    int nodes_size = 0;

};

#include "RRTGraph.cpp"

#endif
