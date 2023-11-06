#ifndef MODULES_MOTION2DPOSITIONVELOCITYSINGLELINKED_H
#define MODULES_MOTION2DPOSITIONVELOCITYSINGLELINKED_H

#include "Motion2DInterfaceSingle.h"
#include "Motion1DInterfaceSingle.h"
#include "Motion1DPositionVelocitySingle.h"
#include "Motion1DPositionVelocitySingleTimed.h"

class Motion2DPositionVelocitySingleLinked : public Motion2DInterfaceSingle {

public:

    void configure_position_limits(float _limit_x_neg, float _limit_y_neg, float _limit_x_pos, float _limit_y_pos);
    void configure_velocity_limits(float _limit);
    void configure_acceleration_limits(float _limit);
    void configure_jerk_limits(float _limit);
    void configure_dt(float _dt);

    void jump_to(float _pos_x, float _pos_y, float _vel_x = 0, float _vel_y = 0, float _acc_x = 0, float _acc_y = 0);
    void set_arrival_time(float _arrival_time);
    void set_target(float _pos_x = NAN, float _pos_y = NAN, float _vel_x = NAN, float _vel_y = NAN, float _acc_x = NAN, float _acc_y = NAN);

    void run_next_timestep();
    void reset_state();

    coords_2d_s get_position();
    coords_2d_s get_velocity();
    coords_2d_s get_acceleration();
    coords_2d_s get_jerk();
    float get_time_remaining();
    motion_state_s get_state_at_time(float t);
    uint64_t get_error_code();
    coords_2d_s get_debug(int index);

private:

    Motion1DInterfaceSingle* x = (Motion1DInterfaceSingle*) new Motion1DPositionVelocitySingle();
    Motion1DInterfaceSingle* y = (Motion1DInterfaceSingle*) new Motion1DPositionVelocitySingle();
    Motion1DInterfaceSingle* xt = (Motion1DInterfaceSingle*) new Motion1DPositionVelocitySingleTimed();
    Motion1DInterfaceSingle* yt = (Motion1DInterfaceSingle*) new Motion1DPositionVelocitySingleTimed();

    float cur_arrival_time = 0;

    motion_state_s user;
    float user_arrival_time = 0;

    float dt = 1;

    coords_2d_s debug[6] = {0};
};

#endif //MODULES_MOTION2DPOSITIONVELOCITYSINGLELINKED_H
