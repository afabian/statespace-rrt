#ifndef RRT_OUTPUT_CPP
#define RRT_OUTPUT_CPP

#include "RRT.h"
#include "Utils.h"

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::configureDebugOutput(bool _sampling, bool _rewire, std::string _filename_prefix, int width, int height) {
    sampling_output_enabled = _sampling;
    rewire_output_enabled = _rewire;
    debug_output_prefix = _filename_prefix;
    map->configureVis(width, height);
    mkpath(debug_output_prefix.c_str(), S_IRWXU);
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::renderVis() {

    // clear the output
    map->resetVis();

    // add all of the edges
    Node<State>* node = graph.first();
    while (node != nullptr) {
        if (node->parent != nullptr) {
            map->addVisLine(&node->parent->state, &node->state, 0x0000bbbb);
        }
        node = node->next;
    }

    // add all of the points
    node = graph.first();
    while (node != nullptr) {
        map->addVisPoint(&node->state, 0x00006600, true);
        node = node->next;
    }

    // add the goal path
    if (goal.parent != nullptr) {
        node = goal.parent;
        int goal_path_length = 0;
        while (node != start) {
            Node<State>* parent = node->parent;
            map->addVisLine(&parent->state, &node->state, 0x00ff0000);
            map->addGoalDetail(&parent->state, &node->state);
            node = parent;
            if (++goal_path_length > 1000) break;
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
