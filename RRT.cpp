#ifndef RRT_CPP
#define RRT_CPP

#include "RRT.h"
#include <cfloat>
#include <cmath>
#include <iostream>

template <class State, class StateMath, class Map>
void RRT<State,StateMath,Map>::setStartState(State* state) {
    start = graph.addNode(state);
    start->cost = 0;
}

template <class State, class StateMath, class Map>
void RRT<State,StateMath,Map>::setGoalState(State* state) {
    goal.state = *state;
    goal.cost = INFINITY;
    goal.parent = nullptr;
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::setMap(Map *_map) {
    map = _map;
    State minimums, maximums;
    map->getBounds(&minimums, &maximums);
    state_math.setRandomStateConstraints(minimums, maximums);
}

template <class State, class StateMath, class Map>
Node<State>* RRT<State,StateMath,Map>::getNearestNode(State* state) {
    float best_distance = DBL_MAX;
    Node<State>* best_node = nullptr;
    Node<State>* node = graph.first();
    while (node != nullptr) {
        float distance = state_math.distance(state, &node->state);
        if (distance < best_distance) {
            best_distance = distance;
            best_node = node;
        }
        node = node->next;
    }
    return best_node;
}

template <class State, class StateMath, class Map>
void RRT<State,StateMath,Map>::runOnce() {
    State candidate;
    Node<State>* nearest = nullptr;
    Node<State>* newnode = nullptr;

    goal_distance_threshold = calc_goal_distance_threshold();
    neighborhood_distance_threshold = calc_neighborhood_distance_threshold();

    bool suitable_found = false;
    while (!suitable_found) {
        candidate = state_math.getRandomState();
        nearest = getNearestNode(&candidate);
        if (nearest != nullptr) {
            if (!map->edgeInObstacle(&candidate, &nearest->state)) {
                float cost = nearest->cost + state_math.distance(&candidate, &nearest->state);
                if (cost < goal.cost) /* dont add nodes if they're too costly to be a best solution */ {
                    newnode = graph.addNode(&candidate);
                    newnode->parent = nearest;
                    newnode->cost = cost;
                    //rewire(newnode);
                    if (state_math.distance(&newnode->state, &goal.state) < goal_distance_threshold) {
                        float goal_cost = newnode->cost + state_math.distance(&newnode->state, &goal.state);
                        if (goal_cost < goal.cost) {
                            goal.cost = goal_cost;
                            goal.parent = newnode;
                            delete_high_cost_nodes(goal.cost);
                        }
                    }
                    suitable_found = true;
                }
            }
        }
    }
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::rewireAll() {
    Node<State>* node = graph.first();
    while (node != nullptr) {
        rewire(node);
        node = node->next;
    }
    rewire(&goal);
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::rewire(Node<State> *target) {
    if (target->parent == nullptr) return;
    Node<State>* node = graph.first();
    while (node != nullptr) {
        float distance = state_math.distance(&node->state, &target->state);
        if (distance < neighborhood_distance_threshold) {
            if (node->cost < target->parent->cost) {
                if (!map->edgeInObstacle(&node->state, &target->state)) {
                    target->parent = node;
                    float new_cost = target->parent->cost + state_math.distance(&target->state, &target->parent->state);
                    float cost_delta = new_cost - target->cost;
                    apply_cost_delta_recursive(target, cost_delta);
                }
            }
        }
        node = node->next;
    }
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::apply_cost_delta_recursive(Node<State> *node, float cost_delta) {
    node->cost += cost_delta;
    Node<State>* i = graph.first();
    while (i != nullptr) {
        if (i->parent == node) {
            apply_cost_delta_recursive(i, cost_delta);
        }
        i = i->next;
    }
    if (goal.parent == node) {
        apply_cost_delta_recursive(&goal, cost_delta);
    }
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::delete_high_cost_nodes(float cost_threshold) {
    Node<State>* node = graph.first();
    while (node != nullptr) {
        if (node->cost > cost_threshold) {
            graph.delNode(node);
        }
        node = node->next;
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
    return full_distance * NEIGHBORHOOD_THRESHOLD_PERCENT;
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