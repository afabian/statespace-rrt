#include "Motion2DPositionVelocityAccelJerkSingleIndependent.h"

void Motion2DPositionVelocityAccelJerkSingleIndependent::configure_position_limits(float _limit_x_neg, float _limit_y_neg, float _limit_x_pos, float _limit_y_pos) {
    x.configure_position_limits(_limit_x_neg, _limit_x_pos);
    y.configure_position_limits(_limit_y_neg, _limit_y_pos);
}

void Motion2DPositionVelocityAccelJerkSingleIndependent::configure_velocity_limits(float _limit) {
    x.configure_velocity_limits(-_limit, _limit);
    y.configure_velocity_limits(-_limit, _limit);
}

void Motion2DPositionVelocityAccelJerkSingleIndependent::configure_acceleration_limits(float _limit) {
    x.configure_acceleration_limits(-_limit, _limit);
    y.configure_acceleration_limits(-_limit, _limit);
}

void Motion2DPositionVelocityAccelJerkSingleIndependent::configure_jerk_limits(float _limit) {
    x.configure_jerk_limits(_limit);
    y.configure_jerk_limits(_limit);
}

void Motion2DPositionVelocityAccelJerkSingleIndependent::configure_dt(float _dt) {
    x.configure_dt(_dt);
    y.configure_dt(_dt);
}

void Motion2DPositionVelocityAccelJerkSingleIndependent::jump_to(float _pos_x, float _pos_y, float _vel_x, float _vel_y, float _acc_x, float _acc_y) {
    x.jump_to(_pos_x, _vel_x, _acc_x);
    y.jump_to(_pos_y, _vel_y, _acc_y);
}

void Motion2DPositionVelocityAccelJerkSingleIndependent::set_arrival_time(float _arrival_time) { }

void Motion2DPositionVelocityAccelJerkSingleIndependent::set_target(float _pos_x, float _pos_y, float _vel_x, float _vel_y, float _acc_x, float _acc_y) {
    x.set_target(_pos_x, _vel_x, _acc_x);
    y.set_target(_pos_y, _vel_y, _acc_y);
}

void Motion2DPositionVelocityAccelJerkSingleIndependent::run_next_timestep() {
    x.run_next_timestep();
    y.run_next_timestep();
}

void Motion2DPositionVelocityAccelJerkSingleIndependent::reset_state() {
    x.reset_state();
    y.reset_state();
}

Motion2DInterfaceSingle::coords_2d_s Motion2DPositionVelocityAccelJerkSingleIndependent::get_position() {
    Motion2DInterfaceSingle::coords_2d_s output;
    output.x = x.get_position();
    output.y = y.get_position();
    return output;
}

Motion2DInterfaceSingle::coords_2d_s Motion2DPositionVelocityAccelJerkSingleIndependent::get_velocity() {
    Motion2DInterfaceSingle::coords_2d_s output;
    output.x = x.get_velocity();
    output.y = y.get_velocity();
    return output;
}

Motion2DInterfaceSingle::coords_2d_s Motion2DPositionVelocityAccelJerkSingleIndependent::get_acceleration() {
    Motion2DInterfaceSingle::coords_2d_s output;
    output.x = x.get_acceleration();
    output.y = y.get_acceleration();
    return output;
}

Motion2DInterfaceSingle::coords_2d_s Motion2DPositionVelocityAccelJerkSingleIndependent::get_jerk() {
    Motion2DInterfaceSingle::coords_2d_s output;
    output.x = x.get_jerk();
    output.y = y.get_jerk();
    return output;
}

float Motion2DPositionVelocityAccelJerkSingleIndependent::get_time_remaining() {
    float x_time = x.get_time_remaining();
    float y_time = y.get_time_remaining();
    float max_time = (std::isnan(y_time) || y_time < x_time) ? x_time : y_time;
    return max_time;
}

Motion2DInterfaceSingle::motion_state_s Motion2DPositionVelocityAccelJerkSingleIndependent::get_state_at_time(float t) {
    Motion1DInterfaceSingle::motion_state_s x_state = x.get_state_at_time(t);
    Motion1DInterfaceSingle::motion_state_s y_state = y.get_state_at_time(t);
    Motion2DInterfaceSingle::motion_state_s output;
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

uint64_t Motion2DPositionVelocityAccelJerkSingleIndependent::get_error_code() {
    uint64_t output = y.get_error_code().value << 32 | x.get_error_code().value;
    return output;
}

Motion2DInterfaceSingle::coords_2d_s Motion2DPositionVelocityAccelJerkSingleIndependent::get_debug(int index) {
    Motion2DInterfaceSingle::coords_2d_s output;
    output.x = x.get_debug(index);
    output.y = y.get_debug(index);
    return output;
}