#include "StateRacerMath.h"
#include <cmath>
#include <iostream>

using namespace std;

///////////////////////////////////////////////  SETUP  //////////////////////////////////////////////////

StateRacerMath::StateRacerMath() { }

void StateRacerMath::setMax(float _V_MAX, float _T_MAX, float _X_MAX, float _Y_MAX) {
    V_MAX = _V_MAX;
    T_MAX = _T_MAX;
    X_MAX = _X_MAX;
    Y_MAX = _Y_MAX;
}

void StateRacerMath::setRes(int _LUT_V_RES, int _LUT_X_RES, int _LUT_Y_RES) {
    LUT_V_RES = _LUT_V_RES;
    LUT_X_RES = _LUT_X_RES;
    LUT_Y_RES = _LUT_Y_RES;
}

void StateRacerMath::setSteps(int _V_STEPS, int _A_STEPS, int _S_STEPS, int _T_STEPS) {
    V_STEPS = _V_STEPS;
    A_STEPS = _A_STEPS;
    S_STEPS = _S_STEPS;
    T_STEPS = _T_STEPS;
}

void StateRacerMath::setMap(MapRacer *_map) {
    map = _map;
    StateRacer _minimums, _maximums;
    map->getBounds(&_minimums, &_maximums);
    setRandomStateConstraints(_minimums, _maximums);
    if (model != nullptr && map != nullptr) {
        generateStateTransitionLUT();
    }
}

void StateRacerMath::setModel(ModelRacer *_model) {
    model = _model;
    if (model != nullptr && map != nullptr) {
        generateStateTransitionLUT();
    }
}

void StateRacerMath::setVis(std::string outputPath) {
    vis.setOutputPath(outputPath);
}

////////////////////////////////////////// MODEL SIMULATION //////////////////////////////////////////////

int StateRacerMath::lutindex(float v, float x, float y) {
    int vidx = (v + V_MAX) / (2 * V_MAX) * LUT_V_RES;
    bool vidx_ok = vidx >= 0 && vidx < LUT_V_RES;
    int xidx = (x + X_MAX) / (2 * X_MAX) * LUT_X_RES;
    bool xidx_ok = xidx >= 0 && xidx < LUT_X_RES;
    int yidx = (y + Y_MAX) / (2 * Y_MAX) * LUT_Y_RES;
    bool yidx_ok = yidx >= 0 && yidx < LUT_Y_RES;
    int idx = vidx * LUT_X_RES * LUT_Y_RES
            + xidx * LUT_Y_RES
            + yidx;
    return vidx_ok && xidx_ok && yidx_ok ? idx : -1;
}

void StateRacerMath::generateStateTransitionLUT() {
    // do forward simulations of the model, iterating over starting states and possible internal control input
    // to generate a map of output states vs. input states.
    // the main RRT loop will use this map to figure out if two states are connectable, and what the cost is.

    StateRacer final;

    delete [] lut;
    lut = new ModelRacerEdgeCost[LUT_V_RES * LUT_X_RES * LUT_Y_RES]{0};

    for (float vi = 0; vi < V_MAX; vi += V_MAX / V_STEPS) {
        for (float accel = -1; accel < 1; accel += 2.0f / A_STEPS) {
            float gas = accel > 0 ? accel : 0;
            float brake = accel < 0 ? -accel : 0;
            for (float steering = -1; steering < 1; steering += 2.0f / S_STEPS) {
                model->reset();
                model->setInitialState(vi);
                model->setControls(gas, brake, steering);
                float dt = T_MAX / T_STEPS;
                for (float t=0; t<T_MAX; t+= dt) {
                    model->run(dt);
                    model->getState(&final);
                    int idx = lutindex(final.v, final.x, final.y);
                    if (idx != -1) {
                        ModelRacerEdgeCost *cost = &lut[idx];
                        cost->brake = brake;
                        cost->gas = gas;
                        cost->steering = steering;
                        cost->dt = t;
                        cost->vf = final.v;
                        cost->hf = final.h;
                        cost->cost = t; // cost function is time
                    }
                }
            }
        }
    }

    // todo: fill in gaps in the table???

    // Visualize
    vis.renderLUT(lut, LUT_V_RES, LUT_X_RES, LUT_Y_RES);

    // is LUT complete?

    bool gap_found = false;
    for (int v=0; v<LUT_V_RES; v++) {
        for (int x=0; x<LUT_X_RES; x++) {
            for (int y=0; y<LUT_Y_RES; y++) {
                int idx = lutindex(v, x, y);
                if (lut[idx].cost == 0) {
                    gap_found = true;
                }
            }
        }
    }

    if (gap_found) {
        cout << "Error: Gaps found in LUT" << endl;
        exit(1);
    }
}

////////////////////////////////////////  OBSTACLE DETECTION  ////////////////////////////////////////////

bool StateRacerMath::pointInObstacle(StateRacer *point) {
    return map->getPixelIsObstacle(point->x, point->y);
}

bool StateRacerMath::edgeInObstacle(StateRacer *source, StateRacer *dest) {
    StateRacer diff(dest->x - source->x, dest->y - source->y);
    float step = EDGE_WALK_SCALE / hypotf(diff.x, diff.y);
    for (float progress = 0; progress < 1; progress += step) {
        StateRacer point(source->x + diff.x * progress, source->y + diff.y * progress);
        if (pointInObstacle(&point)) {
            return true;
        }
    }
    return false;
}

/////////////////////////////////////////  COST CALCULATIONS  ////////////////////////////////////////////

float StateRacerMath::edgeCost(StateRacer *source, StateRacer *dest) {
}

void StateRacerMath::edgePath(StateRacer *source, StateRacer *dest, float t[], float p[], float a[], float pointCount) {
}

///////////////////////////////////////  DISTANCE CALCULATIONS  //////////////////////////////////////////

double StateRacerMath::distance(StateRacer *source, StateRacer *dest) {
    double dx = source->x - dest->x;
    double dy = source->y - dest->y;
    double dist = hypot(dx, dy);
    return dist;
}

double StateRacerMath::approx_distance(StateRacer *source, StateRacer *dest) {
    return fabs(dest->x - source->x) + fabs(dest->y - source->y);
}

////////////////////////////////////////// SAMPLE GENERATION /////////////////////////////////////////////

void StateRacerMath::setRandomStateConstraints(StateRacer _minimums, StateRacer _maximums) {
    minimums = _minimums;
    maximums = _maximums;
    // scale and shift are optimized to make getRandomState() fast
    scale.x = (maximums.x - minimums.x - 1) / RAND_MAX;
    scale.y = (maximums.y - minimums.y - 1) / RAND_MAX;
    scale.v = V_MAX;
    scale.h = 360;
    shift.x = minimums.x;
    shift.y = minimums.y;
    shift.v = 0;
    shift.h = 0;
}

StateRacer StateRacerMath::getRandomState() {
    StateRacer output;
    output.x = (double)rand() * scale.x + shift.x;
    output.y = (double)rand() * scale.y + shift.y;
    output.v = (double)rand() * scale.v + shift.v;
    output.h = (double)rand() * scale.h + shift.h;
    return output;
}
