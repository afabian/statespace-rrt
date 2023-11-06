#include "Motion2DPositionVelocityDoubleLinked.h"

void Motion2DPositionVelocityDoubleLinked::configure_position_limits(double _limit_x_neg, double _limit_y_neg, double _limit_x_pos, double _limit_y_pos) {
    x.configure_position_limits(_limit_x_neg, _limit_x_pos);
    y.configure_position_limits(_limit_y_neg, _limit_y_pos);
    xt.configure_position_limits(_limit_x_neg, _limit_x_pos);
    yt.configure_position_limits(_limit_y_neg, _limit_y_pos);
}

void Motion2DPositionVelocityDoubleLinked::configure_velocity_limits(double _limit) {
    x.configure_velocity_limits(-_limit, _limit);
    y.configure_velocity_limits(-_limit, _limit);
    xt.configure_velocity_limits(-_limit, _limit);
    yt.configure_velocity_limits(-_limit, _limit);
}

void Motion2DPositionVelocityDoubleLinked::configure_acceleration_limits(double _limit) {
    x.configure_acceleration_limits(-_limit, _limit);
    y.configure_acceleration_limits(-_limit, _limit);
    xt.configure_acceleration_limits(-_limit, _limit);
    yt.configure_acceleration_limits(-_limit, _limit);
}

void Motion2DPositionVelocityDoubleLinked::configure_jerk_limits(double _limit) {
    x.configure_jerk_limits(_limit);
    y.configure_jerk_limits(_limit);
    xt.configure_jerk_limits(_limit);
    yt.configure_jerk_limits(_limit);
}

void Motion2DPositionVelocityDoubleLinked::configure_dt(double _dt) {
    x.configure_dt(_dt);
    y.configure_dt(_dt);
    xt.configure_dt(_dt);
    yt.configure_dt(_dt);
    dt = _dt;
}

void Motion2DPositionVelocityDoubleLinked::jump_to(double _pos_x, double _pos_y, double _vel_x, double _vel_y, double _acc_x, double _acc_y) {
    xt.jump_to(_pos_x, _vel_x, _acc_x);
    yt.jump_to(_pos_y, _vel_y, _acc_y);
}

void Motion2DPositionVelocityDoubleLinked::set_arrival_time(double _arrival_time) {
    user_arrival_time = _arrival_time;
}

void Motion2DPositionVelocityDoubleLinked::set_target(double _pos_x, double _pos_y, double _vel_x, double _vel_y, double _acc_x, double _acc_y) {
    user.pos.x = _pos_x;
    user.pos.y = _pos_y;
    user.vel.x = _vel_x;
    user.vel.y = _vel_y;
    user.acc.x = _acc_x;
    user.acc.y = _acc_y;
    user.jrk.x = 0;
    user.jrk.y = 0;
    xt.set_target(_pos_x, _vel_x, _acc_x);
    yt.set_target(_pos_y, _vel_y, _acc_y);
}

void Motion2DPositionVelocityDoubleLinked::run_next_timestep() {

    // if we're in timed mode, use the given time target.
    // this may cause combined x/y jerk to exceed limits

    if (user_arrival_time) {
        cur_arrival_time = user_arrival_time;
    }

        // if we're not in timed mode, then figure out what the fastest possible arrival time is,
        // and then derate that so that combined x and y outputs dont exceed the output limits.

    else {
        x.jump_to(xt.get_position(), xt.get_velocity(), xt.get_acceleration());
        x.set_target(user.pos.x, user.vel.x, user.acc.x);
        x.run_next_timestep();
        double x_arrival_time = x.get_time_remaining();

        y.jump_to(yt.get_position(), yt.get_velocity(), yt.get_acceleration());
        y.set_target(user.pos.y, user.vel.y, user.acc.y);
        y.run_next_timestep();
        double y_arrival_time = y.get_time_remaining();

        cur_arrival_time = max(x_arrival_time, y_arrival_time);
    }

    // now that we definitely have an arrival time, solve x and y for that time

    xt.set_arrival_time(cur_arrival_time);
    xt.run_next_timestep();

    yt.set_arrival_time(cur_arrival_time);
    yt.run_next_timestep();

    // decrement the user arrival time
    user_arrival_time = max(0.0, user_arrival_time - dt);

}

void Motion2DPositionVelocityDoubleLinked::reset_state() {
    x.reset_state();
    y.reset_state();
    xt.reset_state();
    yt.reset_state();
}

Motion2DInterfaceDouble::coords_2d_s Motion2DPositionVelocityDoubleLinked::get_position() {
    coords_2d_s output;
    output.x = xt.get_position();
    output.y = yt.get_position();
    return output;
}

Motion2DInterfaceDouble::coords_2d_s Motion2DPositionVelocityDoubleLinked::get_velocity() {
    coords_2d_s output;
    output.x = xt.get_velocity();
    output.y = yt.get_velocity();
    return output;
}

Motion2DInterfaceDouble::coords_2d_s Motion2DPositionVelocityDoubleLinked::get_acceleration() {
    coords_2d_s output;
    output.x = xt.get_acceleration();
    output.y = yt.get_acceleration();
    return output;
}

Motion2DInterfaceDouble::coords_2d_s Motion2DPositionVelocityDoubleLinked::get_jerk() {
    coords_2d_s output;
    output.x = xt.get_jerk();
    output.y = yt.get_jerk();
    return output;
}

double Motion2DPositionVelocityDoubleLinked::get_time_remaining() {
    return cur_arrival_time;
}

Motion2DInterfaceDouble::motion_state_s Motion2DPositionVelocityDoubleLinked::get_state_at_time(double t) {
    Motion1DInterfaceDouble::motion_state_s x_state = xt.get_state_at_time(t);
    Motion1DInterfaceDouble::motion_state_s y_state = yt.get_state_at_time(t);
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

uint64_t Motion2DPositionVelocityDoubleLinked::get_error_code() {
    uint64_t output = yt.get_error_code().value << 32 | xt.get_error_code().value;
    return output;
}

Motion2DInterfaceDouble::coords_2d_s Motion2DPositionVelocityDoubleLinked::get_debug(int index) {
    return debug[index];
}
