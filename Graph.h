#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <map>
#include <string>

template <class State>
class Graph {

public:
    int addNode(State* _state);
    void delNode(int _id);

    void addEdge(int _idSource, int _idDest);
    void delEdge(int _idSource, int _idDest);
    std::vector<int>* getLinkedNodes(int _idSource);

    State* atIndex(int _index);
    State* atID(int _id);

    int find(State* _state);
    int size();

    std::string toString();

private:
    std::vector<State> nodes;
    std::vector<std::vector<int>> edges;
    std::vector<int> ids;
    std::map<int, int> indexes;
    int last_id = 0;
};

#include "Graph.cpp"

#endif
