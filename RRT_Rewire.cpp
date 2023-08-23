#ifndef RRT_REWIRE_CPP
#define RRT_REWIRE_CPP

#include "RRT.h"

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::configureRewiring(bool _enabled, float _neighborhood_threshold_percent, int _passes) {
    rewiring_enabled = _enabled;
    rewiring_neighbor_threshold = _neighborhood_threshold_percent;
    rewiring_passes = _passes;
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::rewireAll() {
    Node<State>* node = graph.first();
    while (node != nullptr) {
        rewireNode(node);
        node = node->next;
    }
    rewireNode(&goal);
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::rewireNode(Node<State> *target) {
    if (target->parent == nullptr) return;
    Node<State>* node = graph.first();
    while (node != nullptr) {
        float approx_distance = state_math.approx_distance(&node->state, &target->state);
        if (approx_distance < neighborhood_distance_threshold) {
            float new_cost = node->cost + state_math.distance(&target->state, &node->state);
            if (new_cost < target->cost) {
                if (!map->edgeInObstacle(&node->state, &target->state)) {
                    target->parent = node;
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
void RRT<State, StateMath, Map>::debugOutputRewire(int i) {
    if (rewire_output_enabled) {
        renderVis();
        map->renderVis(debug_output_prefix + "/rewire_" + to_string(i) + ".png");
    }
}

#endif
