#ifndef RRT_H
#define RRT_H

#include "RRTGraph.h"
#include <string>
#include <cmath>

const float GOAL_THRESHOLD_PERCENT = 0.01f;
const float NEIGHBORHOOD_THRESHOLD_PERCENT = 0.25f;

template <class State, class StateMath, class Map>
class RRT {

public:
    void setStartState(State* state);
    void setGoalState(State* state);
    void setMap(Map* _map);
    void runOnce();
    void renderVis();
    std::string getDebugText();
    float getGoalCost();
    void rewireAll();

private:
    Node<State>* getNearestNode(State* state);
    float calc_goal_distance_threshold();
    float calc_neighborhood_distance_threshold();
    void delete_high_cost_nodes(float cost_threshold);
    void rewire(Node<State>* target);
    void apply_cost_delta_recursive(Node<State>* node, float cost_delta);

    Map* map = nullptr;
    
    RRTGraph<State> graph;

    Node<State>* start = nullptr;
    Node<State> goal;

    StateMath state_math;

    float goal_distance_threshold = 0;
    float neighborhood_distance_threshold = 0;

};

#include "RRT.cpp"

#endif
