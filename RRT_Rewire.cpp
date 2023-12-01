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
    for (Node<State>* node = graph.first(); node != nullptr; node = node->next) {
        rewireNode(node);
    }
    rewireNode(&goal);
}

// look for lower-cost parents than the node's current parent.
// if one is found, make it the nodes parent and then recalculate the node and its childrens' costs.
template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::rewireNode(Node<State> *target) {
    if (target->parent == nullptr) return;
    for (Node<State>* node = graph.first(); node != nullptr; node = node->next) {
        float approx_distance = state_math->approx_distance(&node->state, &target->state);
        if (node != target) {
            if (approx_distance < neighborhood_distance_threshold) {
                float edge_cost = state_math->edgeCost(&node->state, &target->state);
                float new_cost = node->cost + edge_cost;
                if (new_cost < target->cost) {
                    if (!state_math->edgeInObstacle(&node->state, &target->state)) {
                        if (target == &goal) { // TODO: TEMP
                            target->parent = target->parent;
                        }
                        target->parent = node;
                        float cost_delta = new_cost - target->cost;
                        apply_cost_delta_recursive(target, cost_delta);
                    }
                }
            }
        }
    }
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::apply_cost_delta_recursive(Node<State> *root, float cost_delta) {
    root->cost += cost_delta;
    for (Node<State>* node = graph.first(); node != nullptr; node = node->next) {
        if (node->parent == root && node != root) {
            apply_cost_delta_recursive(node, cost_delta);
        }
    }
    if (goal.parent == root && &goal != root) {
        apply_cost_delta_recursive(&goal, cost_delta);
    }
}

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::debugOutputRewire(int i) {
    if (rewire_output_enabled) {
        renderVis();
        map->renderVis(debug_output_prefix + "/rewire_" + to_string(i));
    }
}

#endif
