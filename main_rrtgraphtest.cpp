#include "RRTGraph.h"
#include "statespace/2d/State2D.h"

#include <iostream>

using namespace std;

int main(int argc, char* argv[]) {

    RRTGraph<State2D> graph;
    Node<State2D>* node0;
    Node<State2D>* node1;
    Node<State2D>* node2;
    Node<State2D>* node3;
    Node<State2D>* node4;
    Node<State2D>* node5;
    cout << graph.toString() << endl;

    State2D state;

    state.set(10, 11);
    cout << "graph.addNode(State2D(10, 11))" << endl;
    node0 = graph.addNode(&state);
    cout << graph.toString() << endl;

    state.set(20, 21);
    cout << "graph.addNode(State2D(20, 21))" << endl;
    node1 = graph.addNode(&state);
    cout << graph.toString() << endl;

    cout << "node1->cost=6.66" << endl;
    node1->cost = 6.66;
    cout << graph.toString() << endl;

    state.set(30, 31);
    cout << "graph.addNode(State2D(30, 31))" << endl;
    node2 = graph.addNode(&state);
    cout << graph.toString() << endl;

    state.set(40, 41);
    cout << "graph.addNode(State2D(40, 41))" << endl;
    node3 = graph.addNode(&state);
    cout << graph.toString() << endl;

    state.set(50, 51);
    cout << "graph.addNode(State2D(50, 51))" << endl;
    node4 = graph.addNode(&state);
    cout << graph.toString() << endl;

    cout << "graph.delNode(node2)" << endl;
    graph.delNode(node2);
    cout << graph.toString() << endl;

    state.set(60, 61);
    cout << "graph.addNode(State2D(60, 61))" << endl;
    node5 = graph.addNode(&state);
    cout << graph.toString() << endl;

    cout << "node2->parent = node1" << endl;
    node2->parent = node1;
    cout << graph.toString() << endl;

    cout << "node3->parent = node1" << endl;
    node3->parent = node1;
    cout << graph.toString() << endl;

    cout << "node4->parent = node3" << endl;
    node4->parent = node3;
    cout << graph.toString() << endl;

    cout << "graph.delNode(4)" << endl;
    graph.delNode(node4);
    cout << graph.toString() << endl;

    state.set(20, 21);
    cout << "graph.find(State2D(20, 21))" << endl;
    Node<State2D>* node = graph.find(&state);
    cout << "Found state at node " << node << endl;

    cout << "node1->cost" << endl;
    cout << node1->cost << endl;

}