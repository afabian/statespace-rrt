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
    State candidate, candidate_from_edge_calc;
    Node<State>* nearest = nullptr;
    Node<State>* newnode = nullptr;

    bool suitable_found = false;
    while (!suitable_found) {
        candidate = state_math->getRandomState();
        nearest = getNearestNode(&candidate);
        if (nearest != nullptr) {
            if (!state_math->edgeInObstacle(&nearest->state, &candidate)) {
                // edgeCost can modify candidate if it has parameters that are meant to be set after finding a solution.
                // this is essentially part of the sampling process, but instead of sampling all random values, we sample
                // some random values and then generate the rest based on a working solution.
                // todo: does this indicate a problem?
                float edgecost = state_math->edgeCost(&nearest->state, &candidate, &candidate_from_edge_calc);
                float cost = nearest->cost + edgecost;
                if ((cost < goal.cost || allow_costly_nodes) && cost < INFINITY) {
                    suitable_found = true;
                    newnode = graph.addNode(&candidate_from_edge_calc, nearest, cost);
                    addDebugText("New Node Orig: " + candidate.toString());
                    addDebugText("New Node Calc: " + candidate_from_edge_calc.toString());
                    addDebugText("Neighbor: " + nearest->state.toString());
                    addDebugText("Edge Cost: " + to_string(edgecost));
                    addDebugText("Graph now has " + to_string(graph.size()) + " nodes");
                    addDebugText("");



                    float goal_distance = state_math->distance(&newnode->state, &goal.state);
                    if (goal_distance < goal_distance_threshold) {
                        if (!state_math->edgeInObstacle(&newnode->state, &goal.state)) {
                            float goal_cost = newnode->cost + state_math->edgeCost(&newnode->state, &goal.state);
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
}

template <class State, class StateMath, class Map>
Node<State>* RRT<State,StateMath,Map>::getNearestNode(State* state) {
    float best_distance = DBL_MAX;
    Node<State>* best_node = nullptr;
    for (Node<State>* node = graph.first(); node != nullptr; node = node->next) {
        float distance = state_math->distance(&node->state, state);
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
    float full_distance = state_math->distance(&minimums, &maximums);
    return full_distance * goal_threshold_percent;
}

template <class State, class StateMath, class Map>
float RRT<State,StateMath,Map>::calc_neighborhood_distance_threshold() {
    State minimums, maximums;
    map->getBounds(&minimums, &maximums);
    float full_distance = state_math->distance(&minimums, &maximums);
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
            clearDebugBuffer();
        }
    }
}

#endif
