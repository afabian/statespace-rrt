//
// Created by afabian on 3/8/21.
//

#ifndef MODULES_MOTION2DINTERFACESINGLE_H
#define MODULES_MOTION2DINTERFACESINGLE_H

#include <cmath>
#include <cstdint>
#include "../utils.h"

class Motion2DInterfaceSingle {

public:

	struct coords_2d_s {
		float x;
		float y;
	};

    struct motion_state_s {
    	coords_2d_s pos;
    	coords_2d_s vel;
    	coords_2d_s acc;
    	coords_2d_s jrk;
    };

    // configuration

	virtual void configure_position_limits(float _limit_x_neg, float _limit_y_neg, float _limit_x_pos, float _limit_y_pos) = 0;

    virtual void configure_velocity_limits(float _limit) = 0;

    virtual void configure_acceleration_limits(float _limit) = 0;

    virtual void configure_jerk_limits(float _limit) = 0;

	virtual void configure_dt(float _dt) = 0;

    // input setters

    virtual void jump_to(float _pos_x, float _pos_y, float _vel_x, float _vel_y, float _acc_x, float _acc_y) = 0;

    virtual void set_arrival_time(float _arrival_time) = 0;

    virtual void set_target(float _pos_x, float _pos_y, float _vel_x, float _vel_y, float _acc_x, float _acc_y) = 0;

    // main method

    virtual void run_next_timestep() = 0;

    virtual void reset_state() = 0;

    // output getters

    virtual coords_2d_s get_position() = 0;

    virtual coords_2d_s get_velocity() = 0;

    virtual coords_2d_s get_acceleration() = 0;

    virtual coords_2d_s get_jerk() = 0;

    virtual float get_time_remaining() = 0;

    virtual coords_2d_s get_debug(int index) = 0;

    virtual motion_state_s get_state_at_time(float t) = 0;

    // error reporting

    virtual uint64_t get_error_code() = 0;

};

#endif //MODULES_MOTION2DINTERFACESINGLE_H
