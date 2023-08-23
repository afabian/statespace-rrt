#ifndef RRT_CPP
#define RRT_CPP

#include "RRT.h"

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

template<class State, class StateMath, class Map>
void RRT<State, StateMath, Map>::run() {

    for (int i=0; i<sampling_passes; i++) {
        addRandomSample();
        debugOutputSample(i);
    }

    if (rewiring_enabled) {
        for (int i=0; i<rewiring_passes; i++) {
            rewireAll();
            debugOutputRewire(i);
        }
    }

}

#endif