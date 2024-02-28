//
// Created by afabian on 2/27/2024.
//

#include <cmath>
#include "ModelRacer.h"

ModelRacer::ModelRacer(float _gas_strength, float _brake_strength, float _steering_strength, float _steering_friction, float _air_strength, float _dt_internal) {
    gas_strength = _gas_strength;
    brake_strength = _brake_strength;
    steering_strength = _steering_strength;
    steering_friction = _steering_friction;
    air_strength = _air_strength;
    dt_internal = _dt_internal;
}

void ModelRacer::reset() {
    state.set(0, 0, 0, 0, 0);
}

void ModelRacer::setInitialState(float vi) {
    state.v = vi;
}

void ModelRacer::setControls(float _gas, float _brake, float _steering) {
    gas = _gas;
    brake = _brake;
    steering = _steering;
}

void ModelRacer::getState(StateRacer *_state) {
    _state->set(state);
}

void ModelRacer::run(float dt) {
    float t_last = 0;
    for (float t=0; t<dt; t+= dt_internal) {
        if (t > dt) t = dt;
        float dt_now = t - t_last;
        t_last = t;

        // steering
        state.h += steering * steering_strength / (state.v + 1) * dt_now;

        // gas
        state.v += gas * gas_strength / (state.v + 1) * dt_now;

        // brake
        state.v -= brake * brake_strength * dt_now;
        if (state.v < 0) state.v = 0;

        // air resistance
        state.v -= air_strength * state.v * state.v * dt_now;

        // steering friction
        state.v -= fabsf(steering) * steering_strength * steering_friction * state.v * dt_now;

        // position
        state.x += state.v * cos(state.h) * dt_now;
        state.y += state.v * sin(state.h) * dt_now;

        // time
        state.t += dt_now;
    }
}
