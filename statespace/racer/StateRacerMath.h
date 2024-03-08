#ifndef STATERACERMATH_H
#define STATERACERMATH_H

#include "StateRacer.h"
#include "MapRacer.h"
#include "ModelRacer.h"
#include "ModelRacerEdgeCost.h"
#include "StateRacerMathVis.h"
#include <string>

// Racer is a top-down racing game solver, where the car can accelerate, brake, coast, go straight or left or right.

class MapRacer;

class StateRacerMath {

public:
    StateRacerMath();

    void setMap(MapRacer* _map);
    void setMax(float _V_MAX, float _T_MAX, float _X_MAX, float _Y_MAX);
    void setRes(int _LUT_V_RES, int _LUT_X_RES, int _LUT_Y_RES);
    void setSteps(int _V_STEPS, int _A_STEPS, int _S_STEPS, int _T_STEPS);

    void setModel(ModelRacer* _model);

    void setVis(std::string outputPath);

    bool pointInObstacle(StateRacer* point);
    bool edgeInObstacle(StateRacer* source, StateRacer* dest);

    float edgeCost(StateRacer* source, StateRacer* dest);

    void edgePath(StateRacer *source, StateRacer *dest, float t[], float p[], float a[], float pointCount);

    double distance(StateRacer* source, StateRacer* dest);
    double approx_distance(StateRacer* source, StateRacer* dest);

    void setRandomStateConstraints(StateRacer _minimums, StateRacer _maximums);
    StateRacer getRandomState();

protected:
    void generateStateTransitionLUT();

    StateRacer minimums, maximums;
    StateRacer scale, shift;

    const float EDGE_WALK_SCALE = 1.0f;

    MapRacer* map = nullptr;
    ModelRacer* model = nullptr;

    float V_MAX = 10;
    float T_MAX = 10;
    float X_MAX = 100;
    float Y_MAX = 100;

    int LUT_V_RES = 100;
    int LUT_X_RES = 100;
    int LUT_Y_RES = 100;

    int V_STEPS = 100;
    int A_STEPS = 100;
    int S_STEPS = 100;
    int T_STEPS = 100;

    int lutindex(float v0, float vf, float x, float y);

    ModelRacerEdgeCost* lut = nullptr;

    StateRacerMathVis vis;
};

#endif