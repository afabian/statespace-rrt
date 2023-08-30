#include "Graph.h"
#include "statespace/2d/State2D.h"

#include <iostream>

using namespace std;

int main(int argc, char* argv[]) {

    Graph<State2D> graph;
    cout << graph.toString() << endl;

    State2D state;

    state.set(10, 11);
    cout << "graph.addNode(State2D(10, 11))" << endl;
    graph.addNode(&state);
    cout << graph.toString() << endl;

    state.set(20, 21);
    cout << "graph.addNode(State2D(20, 21))" << endl;
    graph.addNode(&state);
    cout << graph.toString() << endl;

    state.set(30, 31);
    cout << "graph.addNode(State2D(30, 31))" << endl;
    graph.addNode(&state);
    cout << graph.toString() << endl;

    state.set(40, 41);
    cout << "graph.addNode(State2D(40, 41))" << endl;
    graph.addNode(&state);
    cout << graph.toString() << endl;

    state.set(50, 51);
    cout << "graph.addNode(State2D(50, 51))" << endl;
    graph.addNode(&state);
    cout << graph.toString() << endl;

    cout << "graph.delNode(2)" << endl;
    graph.delNode(2);
    cout << graph.toString() << endl;

    cout << "graph.delNode(2)" << endl;
    graph.delNode(2);
    cout << graph.toString() << endl;

    state.set(60, 61);
    cout << "graph.addNode(State2D(60, 61))" << endl;
    graph.addNode(&state);
    cout << graph.toString() << endl;

    cout << "graph.addEdge(1, 2)" << endl;
    graph.addEdge(1, 2);
    cout << graph.toString() << endl;

    cout << "graph.addEdge(1, 3)" << endl;
    graph.addEdge(1, 3);
    cout << graph.toString() << endl;

    cout << "graph.addEdge(1, 4)" << endl;
    graph.addEdge(1, 4);
    cout << graph.toString() << endl;

    cout << "graph.addEdge(3, 1)" << endl;
    graph.addEdge(3, 1);
    cout << graph.toString() << endl;

    cout << "graph.delEdge(1, 3)" << endl;
    graph.delEdge(1, 3);
    cout << graph.toString() << endl;

    state.set(60, 61);
    cout << "graph.find(State2D(60, 61))" << endl;
    int i = graph.find(&state);
    cout << "Found state at index " << i << endl;

    cout << "graph.getLinkedNodes(1)" << endl;
    vector<int>* related = graph.getLinkedNodes(1);
    for (auto j : *related) {
        cout << j << endl;
    }
}