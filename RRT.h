#ifndef RRT_H
#define RRT_H

#include "RRTGraph.h"
#include <string>
#include <cmath>

const float GOAL_THRESHOLD_PERCENT = 0.01f;
const float NEIGHBORHOOD_THRESHOLD_PERCENT_DEFAULT = 0.05f;

template <class State, class StateMath, class Map>
class RRT {

public:
    void setStartState(State* state);
    void setGoalState(State* state);
    void setMap(Map* _map);
    void configureSampling(int _passes);
    void configureRewiring(bool _enabled, float _neighborhood_threshold_percent, int _passes);
    void configureDebugOutput(bool _sampling, bool _rewire, std::string _filename_prefix);
    void run();
    void addRandomSample();
    void renderVis();
    std::string getDebugText();
    float getGoalCost();
    void rewireAll();

private:
    Node<State>* getNearestNode(State* state);
    float calc_goal_distance_threshold();
    float calc_neighborhood_distance_threshold();
    void delete_high_cost_nodes(float cost_threshold);
    void rewireNode(Node<State>* target);
    void apply_cost_delta_recursive(Node<State>* node, float cost_delta);
    void debugOutputSample(int iteration);
    void debugOutputRewire(int iteration);

    Map* map = nullptr;
    
    RRTGraph<State> graph;

    Node<State>* start = nullptr;
    Node<State> goal;

    StateMath state_math;

    float goal_distance_threshold = 0;
    float neighborhood_distance_threshold = 0;

    int sampling_passes = 1;

    bool rewiring_enabled = false;
    float rewiring_neighbor_threshold = NEIGHBORHOOD_THRESHOLD_PERCENT_DEFAULT;
    int rewiring_passes = 1;

    bool sampling_output_enabled = false;
    bool rewire_output_enabled = false;
    std::string debug_output_prefix = "";
};

#include "RRT.cpp"
#include "RRT_Sampling.cpp"
#include "RRT_Rewire.cpp"
#include "RRT_Output.cpp"

#endif
