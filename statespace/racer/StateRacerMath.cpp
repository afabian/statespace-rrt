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
    map->setStateRacerMath(this);
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

int StateRacerMath::lutindex(float v0, float dforwardf, float drightf) {
    int v0idx = int(v0 / V_MAX * float(LUT_V_RES));
    bool v0idx_ok = v0idx >= 0 && v0idx < LUT_V_RES;

    int dforwardidx = int((dforwardf + X_MAX) / (2 * X_MAX) * float(LUT_X_RES));
    bool dforwardidx_ok = dforwardidx >= 0 && dforwardidx < LUT_X_RES;

    int drightidx = int((drightf + Y_MAX) / (2 * Y_MAX) * float(LUT_Y_RES));
    bool drightidx_ok = drightidx >= 0 && drightidx < LUT_Y_RES;

    int idx = v0idx * LUT_X_RES * LUT_Y_RES
            + dforwardidx * LUT_Y_RES
            + drightidx;

    return v0idx_ok && dforwardidx_ok && drightidx_ok ? idx : -1;
}

void StateRacerMath::generateStateTransitionLUT() {
    // do forward simulations of the model, iterating over starting states and possible internal control input
    // to generate a map of output states vs. input states.
    // the main RRT loop will use this map to figure out if two states are connectable, and what the cost is.
    // the table is time, position, and heading invariant, so it looks up data based on initial velocity and final relative position
    // when an entry is found, that entry will provide the final velocity and relative heading

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
                    StateRacer final;
                    model->getState(&final);
                    int idx = lutindex(vi, final.x, final.y);
                    if (idx != -1) {
                        ModelRacerEdgeCost *cost = &lut[idx];
                        cost->brake = brake;
                        cost->gas = gas;
                        cost->steering = steering;
                        cost->dt = t;
                        cost->vf = final.v;
                        cost->hf = final.h;
                        cost->cost = t; // cost function is time

                        // todo: figure out how to turn these points into outlines of solid sections, and paint in those sections
                    }
                }
            }
        }
    }

    // todo: fill in gaps in the table???

    // Visualize
    vis.renderLUT(lut, LUT_V_RES, LUT_X_RES, LUT_Y_RES, V_MAX);

}

////////////////////////////////////////  OBSTACLE DETECTION  ////////////////////////////////////////////

bool StateRacerMath::pointInObstacle(StateRacer *point) {
    return map->getPixelIsObstacle(point->x, point->y);
}

bool StateRacerMath::edgeInObstacle(StateRacer *source, StateRacer *dest) {

    if (source == dest) return false;

    int dist = max(2, int(hypotf(dest->x - source->x, dest->y - source->y) * EDGE_WALK_SCALE));
    StateRacer* points = (StateRacer*)malloc(sizeof(StateRacer) * dist);
    bool path_found = edgePath(source, dest, points, dist);

    if (!path_found) return true;

    bool found = false;
    for (int i=0; i<dist; i++) {
        if (pointInObstacle(&points[i])) {
            found = true;
            break;
        }
    }

    free(points);

    return found;
}

/////////////////////////////////////////  COST CALCULATIONS  ////////////////////////////////////////////

ModelRacerEdgeCost* StateRacerMath::edgeCostObj(StateRacer *source, StateRacer *dest) {
    // tranform this segment so that it's origin is (0, 0) and its initial heading is 0
    // then look up the segment in the LUT

    float heading = atan2(dest->x - source->x, dest->y - source->y);
    float dHeading = heading - source->h;

    double dx = dest->x - source->x;
    double dy = dest->y - source->y;
    double dist = hypot(dx, dy);

    StateRacer dest_relative_to_origin;
    dest_relative_to_origin.x = dist * sin(dHeading);
    dest_relative_to_origin.y = dist * cos(dHeading);

    int idx = lutindex(source->v, dest_relative_to_origin.x, dest_relative_to_origin.y);

    if (idx == -1) return nullptr;
    if (lut[idx].cost == 0) return nullptr;
    return &lut[idx];
}

float StateRacerMath::edgeCost(StateRacer *source, StateRacer *dest, StateRacer *dest_updated) {
    ModelRacerEdgeCost* obj = edgeCostObj(source, dest);
    float output = INFINITY;
    if (obj) {
        float heading = atan2f(dest->x - source->x, dest->y - source->y);
        output= obj->cost;
        if (dest_updated != nullptr) {
            *dest_updated = *dest;
            dest_updated->v = obj->vf;
            dest_updated->h = obj->hf + heading;
        }
    }
    return output;
}

bool StateRacerMath::edgePath(StateRacer *source, StateRacer *dest, StateRacer p[], int pointCount) {
    ModelRacerEdgeCost* obj = edgeCostObj(source, dest);
    if (obj) {
        model->reset();
        model->setInitialState(source);
        model->setControls(obj->gas, obj->brake, obj->steering);
        float dt = obj->cost / float(pointCount);
        for (int i = 0; i < pointCount; i++) {
            model->run(dt);
            model->getState(&p[i]);
        }
    }
    return obj != nullptr;
}

///////////////////////////////////////  DISTANCE CALCULATIONS  //////////////////////////////////////////

double StateRacerMath::distance(StateRacer *source, StateRacer *dest) {
    // TODO: This is just a position distance, doesn't consider if vel+hdg make path possible or not
    // augment or replace with edgeCost() ???

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
    // scale should always be divided by RAND_MAX to prepare for rand() being used as an input in getRandomState()
    scale.x = (maximums.x - minimums.x - 1) / RAND_MAX;
    scale.y = (maximums.y - minimums.y - 1) / RAND_MAX;
    scale.v = V_MAX  / RAND_MAX;
    scale.h = M_PI * 2  / RAND_MAX;
    shift.x = minimums.x;
    shift.y = minimums.y;
    shift.v = 0;
    shift.h = -M_PI;
}

StateRacer StateRacerMath::getRandomState() {
    StateRacer output;
    output.x = (double)rand() * scale.x + shift.x;
    output.y = (double)rand() * scale.y + shift.y;
    // V and H aren't generated for the sample, because our algorithm here is to generate x-y samples and then if they can be connected to a node by any V/H settings, use those
    //output.v = (double)rand() * scale.v + shift.v;
    //output.h = (double)rand() * scale.h + shift.h;
    output.v = 0;
    output.h = 0;
    return output;
}
