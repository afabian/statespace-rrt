#ifndef RRT_CPP
#define RRT_CPP

#include "RRT.h"

template<class State, class StateMath, class Map>
RRT<State, StateMath, Map>::RRT(Map *_map, StateMath *_state_math) {
    map = _map;
    state_math = _state_math;
    state_math->setMap(map);
}

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
void RRT<State, StateMath, Map>::run() {

    initRandomSamples();

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