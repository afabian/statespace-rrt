#ifndef MODULES_MOTION2DPOSITIONVELOCITYACCELDOUBLELINKED_H
#define MODULES_MOTION2DPOSITIONVELOCITYACCELDOUBLELINKED_H

#include "Motion2DInterfaceDouble.h"
#include "Motion1DInterfaceDouble.h"
#include "Motion1DPositionVelocityAccelDouble.h"
#include "Motion1DPositionVelocityAccelDoubleTimed.h"

class Motion2DPositionVelocityAccelDoubleLinked : public Motion2DInterfaceDouble {

public:

    void configure_position_limits(double _limit_x_neg, double _limit_y_neg, double _limit_x_pos, double _limit_y_pos);
    void configure_velocity_limits(double _limit);
    void configure_acceleration_limits(double _limit);
    void configure_jerk_limits(double _limit);
    void configure_dt(double _dt);

    void jump_to(double _pos_x, double _pos_y, double _vel_x = 0, double _vel_y = 0, double _acc_x = 0, double _acc_y = 0);
    void set_arrival_time(double _arrival_time);
    void set_target(double _pos_x = NAN, double _pos_y = NAN, double _vel_x = NAN, double _vel_y = NAN, double _acc_x = NAN, double _acc_y = NAN);

    void run_next_timestep();
    void reset_state();

    coords_2d_s get_position();
    coords_2d_s get_velocity();
    coords_2d_s get_acceleration();
    coords_2d_s get_jerk();
    double get_time_remaining();
    motion_state_s get_state_at_time(double t);
    uint64_t get_error_code();
    coords_2d_s get_debug(int index);

private:

    Motion1DPositionVelocityAccelDouble x;
    Motion1DPositionVelocityAccelDouble y;
    Motion1DPositionVelocityAccelDoubleTimed xt;
    Motion1DPositionVelocityAccelDoubleTimed yt;

    bool target_has_changed_since_last_run();
    void compute_movement_plan();

    Motion1DInterfaceDouble::motion_state_s x_state_at_time_since_beginning = {0};
    Motion1DInterfaceDouble::motion_state_s y_state_at_time_since_beginning = {0};

    double cur_arrival_time = 0;
    double time_since_beginning = 0;

    motion_state_s user = {0};
    motion_state_s last_user = {0};
    double user_arrival_time = 0;

    double dt = 1;

    coords_2d_s debug[6] = {0};

    coords_2d_s limit_vel_pos = {0};
    coords_2d_s limit_vel_neg = {0};
    coords_2d_s limit_acc_pos = {0};
    coords_2d_s limit_acc_neg = {0};
};

#endif //MODULES_MOTION2DPOSITIONVELOCITYACCELDOUBLELINKED_H
