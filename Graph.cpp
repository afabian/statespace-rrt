#ifndef GRAPH_CPP
#define GRAPH_CPP

#include "Graph.h"
#include <sstream>

using namespace std;

template <class State>
int Graph<State>::addNode(State* _state) {
    int node_id = last_id;
    int index = nodes.size();
    nodes.push_back(*_state);
    edges.emplace_back();
    last_id++;
    ids.push_back(node_id);
    indexes[node_id] = index;
    return node_id;
}

template <class State>
void Graph<State>::delNode(int _id) {
    if (indexes.count(_id) > 0) {
        int index = indexes[_id];
        nodes.erase(nodes.begin() + index);
        edges.erase(edges.begin() + index);
        ids.erase(ids.begin() + index);
        indexes.erase(_id);
        for (int i=index, j=ids.size(); i<j; i++) {
            indexes[ids[i]]--;
        }
    }
}

template <class State>
void Graph<State>::addEdge(int _idSource, int _idDest) {
    if (indexes.count(_idSource) == 0) return;
    if (indexes.count(_idDest) == 0) return;
    int indexSource = indexes[_idSource];
    edges[indexSource].push_back(_idDest);
}

template <class State>
void Graph<State>::delEdge(int _idSource, int _idDest) {
    int indexSource = indexes[_idSource];
    for (int i=0; i<edges[indexSource].size(); i++) {
        if (edges[indexSource][i] == _idDest) {
            edges[indexSource].erase(edges[indexSource].begin()+i);
        }
    }
}

template<class State>
std::vector<int> *Graph<State>::getLinkedNodes(int _idSource) {
    if (indexes.count(_idSource) == 0) return nullptr;
    int indexSource = indexes[_idSource];
    return &edges[indexSource];
}

template <class State>
State* Graph<State>::atIndex(int _index) {
    return &nodes.at(_index);
}

template <class State>
State* Graph<State>::atID(int _id) {
    int index = indexes[_id];
    return &nodes.at(index);
}

template <class State>
int Graph<State>::find(State* _state) {
    for (int i=0, j=nodes.size(); i<j; i++) {
        if (nodes.at(i) == *_state) {
            return ids[i];
        }
    }
    return -1;
}

template <class State>
int Graph<State>::size() {
    return nodes.size();
}

template <class State>
string Graph<State>::toString() {
    stringstream output;
    for (int i=0; i<nodes.size(); i++) {
        output << "Node " << ids[i] << ": " << nodes[i].toString();
        if (!edges[i].empty()) {
            output << " -> ";
            for (int j=0; j<edges[i].size(); j++) {
                output << edges[i][j] << " ";
            }
        }
        output << endl;
    }
    return output.str();
}

#endif
