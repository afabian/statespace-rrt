#ifndef MODULES_MOTION2DPOSITIONVELOCITYACCELJERKDOUBLEINDEPENDENT_H
#define MODULES_MOTION2DPOSITIONVELOCITYACCELJERKDOUBLEINDEPENDENT_H

#include "Motion2DInterfaceDouble.h"
#include "Motion1DInterfaceDouble.h"
#include "Motion1DPositionVelocityAccelJerkDouble.h"

class Motion2DPositionVelocityAccelJerkDoubleIndependent : public Motion2DInterfaceDouble {

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

    Motion1DPositionVelocityAccelJerkDouble x;
    Motion1DPositionVelocityAccelJerkDouble y;

};

#endif //MODULES_MOTION2DPOSITIONVELOCITYACCELJERKDOUBLEINDEPENDENT_H
