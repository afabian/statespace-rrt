#include <iostream>
#include "statespace/racer/ModelRacer.h"

using namespace std;

int main(int argc, char* argv[]) {

    float gas_strength = 150;
    float brake_strength = 30;
    float steering_strength = 1;
    float steering_friction = 0.1;
    float air_strength = 0.0012;
    float dt_internal = 0.0001;
    float dt_visible = 0.1;
    ModelRacer model = ModelRacer(gas_strength, brake_strength, steering_strength, steering_friction, air_strength, dt_internal);

    model.reset();

    float v_init = 0;
    model.setInitialState(v_init);

    for (float t=0; t<10; t+=dt_visible) {

        float gas = t < 5 ? 1 : 0;
        float brake = t > 6 ? 1 : 0;
        float steering = 0;
        model.setControls(gas, brake, steering);

        model.run(dt_visible);

        StateRacer state;
        model.getState(&state);
        cout << "t=" << (t+dt_visible) << " " << state.toString() << endl;

    }


}