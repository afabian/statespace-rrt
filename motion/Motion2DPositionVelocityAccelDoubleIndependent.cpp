#include "Motion2DPositionVelocityAccelDoubleIndependent.h"

void Motion2DPositionVelocityAccelDoubleIndependent::configure_position_limits(double _limit_x_neg, double _limit_y_neg, double _limit_x_pos, double _limit_y_pos) {
    x.configure_position_limits(_limit_x_neg, _limit_x_pos);
    y.configure_position_limits(_limit_y_neg, _limit_y_pos);
}

void Motion2DPositionVelocityAccelDoubleIndependent::configure_velocity_limits(double _limit) {
    x.configure_velocity_limits(-_limit, _limit);
    y.configure_velocity_limits(-_limit, _limit);
}

void Motion2DPositionVelocityAccelDoubleIndependent::configure_acceleration_limits(double _limit) {
    x.configure_acceleration_limits(-_limit, _limit);
    y.configure_acceleration_limits(-_limit, _limit);
}

void Motion2DPositionVelocityAccelDoubleIndependent::configure_jerk_limits(double _limit) {
    x.configure_jerk_limits(_limit);
    y.configure_jerk_limits(_limit);
}

void Motion2DPositionVelocityAccelDoubleIndependent::configure_dt(double _dt) {
    x.configure_dt(_dt);
    y.configure_dt(_dt);
}

void Motion2DPositionVelocityAccelDoubleIndependent::jump_to(double _pos_x, double _pos_y, double _vel_x, double _vel_y, double _acc_x, double _acc_y) {
    x.jump_to(_pos_x, _vel_x, _acc_x);
    y.jump_to(_pos_y, _vel_y, _acc_y);
}

void Motion2DPositionVelocityAccelDoubleIndependent::set_arrival_time(double _arrival_time) { }

void Motion2DPositionVelocityAccelDoubleIndependent::set_target(double _pos_x, double _pos_y, double _vel_x, double _vel_y, double _acc_x, double _acc_y) {
    x.set_target(_pos_x, _vel_x, _acc_x);
    y.set_target(_pos_y, _vel_y, _acc_y);
}

void Motion2DPositionVelocityAccelDoubleIndependent::run_next_timestep() {
    x.run_next_timestep();
    y.run_next_timestep();
}

void Motion2DPositionVelocityAccelDoubleIndependent::reset_state() {
    x.reset_state();
    y.reset_state();
}

Motion2DInterfaceDouble::coords_2d_s Motion2DPositionVelocityAccelDoubleIndependent::get_position() {
    Motion2DInterfaceDouble::coords_2d_s output;
    output.x = x.get_position();
    output.y = y.get_position();
    return output;
}

Motion2DInterfaceDouble::coords_2d_s Motion2DPositionVelocityAccelDoubleIndependent::get_velocity() {
    Motion2DInterfaceDouble::coords_2d_s output;
    output.x = x.get_velocity();
    output.y = y.get_velocity();
    return output;
}

Motion2DInterfaceDouble::coords_2d_s Motion2DPositionVelocityAccelDoubleIndependent::get_acceleration() {
    Motion2DInterfaceDouble::coords_2d_s output;
    output.x = x.get_acceleration();
    output.y = y.get_acceleration();
    return output;
}

Motion2DInterfaceDouble::coords_2d_s Motion2DPositionVelocityAccelDoubleIndependent::get_jerk() {
    Motion2DInterfaceDouble::coords_2d_s output;
    output.x = x.get_jerk();
    output.y = y.get_jerk();
    return output;
}

double Motion2DPositionVelocityAccelDoubleIndependent::get_time_remaining() {
    double x_time = x.get_time_remaining();
    double y_time = y.get_time_remaining();
    double max_time = (std::isnan(y_time) || y_time < x_time) ? x_time : y_time;
    return max_time;
}

Motion2DInterfaceDouble::motion_state_s Motion2DPositionVelocityAccelDoubleIndependent::get_state_at_time(double t) {
    Motion1DInterfaceDouble::motion_state_s x_state = x.get_state_at_time(t);
    Motion1DInterfaceDouble::motion_state_s y_state = y.get_state_at_time(t);
    Motion2DInterfaceDouble::motion_state_s output;
    output.pos.x = x_state.pos;
    output.pos.y = y_state.pos;
    output.vel.x = x_state.vel;
    output.vel.y = y_state.vel;
    output.acc.x = x_state.acc;
    output.acc.y = y_state.acc;
    output.jrk.x = x_state.jrk;
    output.jrk.y = y_state.jrk;
    return output;
}

uint64_t Motion2DPositionVelocityAccelDoubleIndependent::get_error_code() {
    uint64_t output = y.get_error_code().value << 32 | x.get_error_code().value;
    return output;
}

Motion2DInterfaceDouble::coords_2d_s Motion2DPositionVelocityAccelDoubleIndependent::get_debug(int index) {
    Motion2DInterfaceDouble::coords_2d_s output;
    output.x = x.get_debug(index);
    output.y = y.get_debug(index);
    return output;
}