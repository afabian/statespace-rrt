#ifndef RRT_OUTPUT_CPP
#define RRT_OUTPUT_CPP

#include "RRT.h"
#include "Utils.h"

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::configureDebugOutput(bool _sampling, bool _rewire, std::string _filename_prefix) {
    sampling_output_enabled = _sampling;
    rewire_output_enabled = _rewire;
    debug_output_prefix = _filename_prefix;
    mkpath(debug_output_prefix.c_str(), S_IRWXU);
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::renderVis() {
    map->resetVis();
    Node<State>* node = graph.first();
    while (node != nullptr) {
        if (node->parent != nullptr) {
            map->addVisLine(&node->state, &node->parent->state, 0x0000bbbb);
        }
        node = node->next;
    }
    node = graph.first();
    while (node != nullptr) {
        map->addVisPoint(&node->state, 0x00006600);
        node = node->next;
    }
    if (goal.parent != nullptr) {
        node = goal.parent;
        while (node != start) {
            Node<State>* parent = node->parent;
            map->addVisLine(&node->state, &parent->state, 0x00ff0000);
            node = parent;
        }
    }
}

template<class State, class StateMath, class Map>
std::string RRT<State, StateMath, Map>::getDebugText() {
    return graph.toString();
}

template<class State, class StateMath, class Map>
float RRT<State, StateMath, Map>::getGoalCost() {
    return goal.cost;
}

#endif
