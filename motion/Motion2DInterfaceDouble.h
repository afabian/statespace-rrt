//
// Created by afabian on 3/8/21.
//

#ifndef MODULES_MOTION2DINTERFACEDOUBLE_H
#define MODULES_MOTION2DINTERFACEDOUBLE_H

#include <cmath>
#include <cstdint>
#include "../utils.h"

class Motion2DInterfaceDouble {

public:

    struct coords_2d_s {
        double x;
        double y;
    };

    struct motion_state_s {
        coords_2d_s pos;
        coords_2d_s vel;
        coords_2d_s acc;
        coords_2d_s jrk;
    };

    // configuration

    virtual void configure_position_limits(double _limit_x_neg, double _limit_y_neg, double _limit_x_pos, double _limit_y_pos) = 0;

    virtual void configure_velocity_limits(double _limit) = 0;

    virtual void configure_acceleration_limits(double _limit) = 0;

    virtual void configure_jerk_limits(double _limit) = 0;

    virtual void configure_dt(double _dt) = 0;

    // input setters

    virtual void jump_to(double _pos_x, double _pos_y, double _vel_x, double _vel_y, double _acc_x, double _acc_y) = 0;

    virtual void set_arrival_time(double _arrival_time) = 0;

    virtual void set_target(double _pos_x, double _pos_y, double _vel_x, double _vel_y, double _acc_x, double _acc_y) = 0;

    // main method

    virtual void run_next_timestep() = 0;

    virtual void reset_state() = 0;

    // output getters

    virtual coords_2d_s get_position() = 0;

    virtual coords_2d_s get_velocity() = 0;

    virtual coords_2d_s get_acceleration() = 0;

    virtual coords_2d_s get_jerk() = 0;

    virtual double get_time_remaining() = 0;

    virtual coords_2d_s get_debug(int index) = 0;

    virtual motion_state_s get_state_at_time(double t) = 0;

    // error reporting

    virtual uint64_t get_error_code() = 0;

};

#endif //MODULES_MOTION2DINTERFACEDOUBLE_H
