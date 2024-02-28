//
// Created by afabian on 2/27/2024.
//

#ifndef RRT_MODELRACER_H
#define RRT_MODELRACER_H

#include "StateRacer.h"

class ModelRacer {

public:
    ModelRacer(float _gas_strength, float _brake_strength, float _steering_strength, float _steering_friction, float _air_strength, float _dt_internal);
    void reset();
    void setInitialState(float vi);
    void setControls(float _gas, float _brake, float _steering);
    void run(float dt);
    void getState(StateRacer* _state);

private:
    // state
    StateRacer state;

    // controls
    float gas = 0;
    float brake = 0;
    float steering = 0;

    // config
    float gas_strength = 0;
    float brake_strength = 0;
    float steering_strength = 0;
    float steering_friction = 0;
    float air_strength = 0;
    float dt_internal = 1;

};


#endif //RRT_MODELRACER_H
