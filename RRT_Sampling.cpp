#ifndef RRT_SAMPLING_CPP
#define RRT_SAMPLING_CPP

#include "RRT.h"

#include <cfloat>

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::configureSampling(int _passes, bool _allow_costly_nodes) {
    sampling_passes = _passes;
    allow_costly_nodes = _allow_costly_nodes;
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::initRandomSamples() {
    goal_distance_threshold = calc_goal_distance_threshold();
    neighborhood_distance_threshold = calc_neighborhood_distance_threshold();
}

template <class State, class StateMath, class Map>
void RRT<State,StateMath,Map>::addRandomSample() {
    State candidate;
    Node<State>* nearest = nullptr;
    Node<State>* newnode = nullptr;

    bool suitable_found = false;
    while (!suitable_found) {
        candidate = state_math.getRandomState();
        nearest = getNearestNode(&candidate);
        if (nearest != nullptr) {
            if (!map->edgeInObstacle(&candidate, &nearest->state)) {
                float cost = nearest->cost + map->edgeCost(&candidate, &nearest->state);
                if (cost < goal.cost || allow_costly_nodes) {
                    suitable_found = true;
                    newnode = graph.addNode(&candidate, nearest, cost);
                    float goal_distance = state_math.distance(&newnode->state, &goal.state);
                    if (goal_distance < goal_distance_threshold) {
                        float goal_cost = newnode->cost + map->edgeCost(&newnode->state, &goal.state);
                        if (goal_cost < goal.cost) {
                            goal.cost = goal_cost;
                            goal.parent = newnode;
                            if (!allow_costly_nodes) {
                                delete_high_cost_nodes(goal.cost);
                            }
                        }
                    }
                }
            }
        }
    }
}

template <class State, class StateMath, class Map>
Node<State>* RRT<State,StateMath,Map>::getNearestNode(State* state) {
    float best_distance = DBL_MAX;
    Node<State>* best_node = nullptr;
    for (Node<State>* node = graph.first(); node != nullptr; node = node->next) {
        float distance = state_math.distance(state, &node->state);
        if (distance < best_distance) {
            best_distance = distance;
            best_node = node;
        }
    }
    return best_node;
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::delete_high_cost_nodes(float cost_threshold) {
    for (Node<State>* node = graph.first(); node != nullptr; node = node->next) {
        if (node->cost > cost_threshold) {
            graph.delNode(node);
        }
    }
}

template <class State, class StateMath, class Map>
float RRT<State,StateMath,Map>::calc_goal_distance_threshold() {
    State minimums, maximums;
    map->getBounds(&minimums, &maximums);
    float full_distance = state_math.distance(&minimums, &maximums);
    return full_distance * GOAL_THRESHOLD_PERCENT;
}

template <class State, class StateMath, class Map>
float RRT<State,StateMath,Map>::calc_neighborhood_distance_threshold() {
    State minimums, maximums;
    map->getBounds(&minimums, &maximums);
    float full_distance = state_math.distance(&minimums, &maximums);
    return full_distance * rewiring_neighbor_threshold;
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::debugOutputSample(int i) {
    if (sampling_output_enabled) {
        if (
                (i < 10) ||
                (i < 100 && i % 10 == 0) ||
                (i < 1000 && i % 100 == 0) ||
                (i < 10000 && i % 1000 == 0) ||
                (i < 100000 && i % 10000 == 0)
                ) {
            renderVis();
            map->renderVis(debug_output_prefix + "/sample_" + to_string(i));
        }
    }
}

#endif
