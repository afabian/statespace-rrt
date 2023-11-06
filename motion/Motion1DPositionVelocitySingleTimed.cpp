#include "Motion1DPositionVelocitySingleTimed.h"

using namespace std;

void Motion1DPositionVelocitySingleTimed::configure_position_limits(float _limit_pos_neg, float _limit_pos_pos) {
    configure_position_limits_int(_limit_pos_neg, _limit_pos_pos);
}
void Motion1DPositionVelocitySingleTimed::configure_velocity_limits(float _limit_vel_neg, float _limit_vel_pos) {
    configure_velocity_limits_int(_limit_vel_neg, _limit_vel_pos);
}

void Motion1DPositionVelocitySingleTimed::configure_acceleration_limits(float _limit_acc_neg, float _limit_acc_pos) {
    configure_acceleration_limits_int(_limit_acc_neg, _limit_acc_pos);
}

void Motion1DPositionVelocitySingleTimed::configure_jerk_limits(float _limit_jrk) {
    configure_jerk_limits_int(_limit_jrk);
}

void Motion1DPositionVelocitySingleTimed::configure_dt(float _dt) {
    configure_dt_int(_dt);
}

void Motion1DPositionVelocitySingleTimed::reset_state() {
    // since there's no constructor / init function, the initial values of these variables must match the below values
    using_position_limit = false;
    position_limit.pos = 0;
    position_limit.vel = 0;
    arrived_at_position_target = false;
}

bool Motion1DPositionVelocitySingleTimed::target_has_changed_since_last_run(motion_state_s* target_state, float target_arrival_time) {
    bool pos_ok = (isnan(target_state->pos) && isnan(last_user_target.pos)) || (target_state->pos == last_user_target.pos);
    bool vel_ok = (isnan(target_state->vel) && isnan(last_user_target.vel)) || (target_state->vel == last_user_target.vel);

    if (!pos_ok || !vel_ok) {
        last_user_target = *target_state;
        last_user_arrival_time = target_arrival_time;
        return true;
    }
    else {
        return false;
    }
}

void Motion1DPositionVelocitySingleTimed::run_next_timestep() {

    // this needs to be accurate as compared to continuous functions, but also stable in discrete time

    // if there are any configuration or user input errors, report them and exit immediately.
    // these must be corrected by fixing the calling code.

    if (!check_input_and_config_errors()) return;

    if (target_has_changed_since_last_run(&user, user_arrival_time)) {
        reset_state();
    }

    // store last values temporary so that they can be used to calculate the new values

    last = cur;

    // compute the new jerk to get motion closer to its target

    float vel = 0;
    if (!isnan(user.pos)) {
        if (!arrived_at_position_target && !using_position_limit) {
            vel = get_vel_for_pos_mode(&last, &user, user_arrival_time);
        } else if (!using_position_limit) {
            vel = isnan(user.vel) ? 0 : user.vel;
        } else {
            vel = get_vel_for_pos_mode(&last, &position_limit, 0);
        }
        cur_time_remaining = using_position_limit ? INFINITY : (arrived_at_position_target ? 0 : get_time_remaining_for(&last, &user, user_arrival_time));
    }
    else {
        if (using_position_limit) {
            vel = get_vel_for_pos_mode(&last, &position_limit, 0);
        } else {
            vel = isnan(user.vel) ? 0 : user.vel;
        }
        cur_time_remaining = using_position_limit ? INFINITY : 0;
    }

    // compute the outputs (other than jerk, which is already set)

    cur = compute_next_state_from_vel(&last, vel, dt);
    user_arrival_time -= dt;

    // compute this jerk and the upcoming stopping jerk, so that the t[] and j[] arrays can be populated accurately

    float jerk_now = (vel - last.vel) / (dt * dt);
    float jerk_stop = (last.vel - vel) / (dt * dt);
    last_solution_j[0] = jerk_now;
    last_solution_j[1] = -jerk_now;
    last_solution_j[2] = 0;
    last_solution_j[3] = jerk_stop;
    last_solution_j[4] = -jerk_stop;
    last_solution_t[1] = dt;
    last_solution_t[2] = dt + dt;
    last_solution_t[3] = cur_time_remaining;
    last_solution_t[4] = cur_time_remaining + dt;
    last_solution_t[5] = cur_time_remaining + dt + dt;

    // if we just arrived at the position target, record that and switch to velocity mode

    if (cur_time_remaining < dt) {
        arrived_at_position_target = true;
    }

    // if we're in velocity mode and about to overrun a position limit, switch to position mode at the limit

    if (!using_position_limit) {
        float next_position = cur.pos + vel * dt;
        if (next_position > limit_pos.pos) {
            position_limit.pos = limit_pos.pos;
            position_limit.vel = 0;
            using_position_limit = true;
        }
        if (next_position < limit_neg.pos) {
            position_limit.pos = limit_neg.pos;
            position_limit.vel = 0;
            using_position_limit = true;
        }
    }

    // check for errors in the output signals

    check_output_errors();

}

float Motion1DPositionVelocitySingleTimed::get_vel_for_pos_mode(motion_state_s* current_state, motion_state_s* target_state, float arrival_time) {
    float vel = (target_state->pos - current_state->pos) / max(dt, arrival_time);
    float vel_limited = minmax(vel, limit_neg.vel, limit_pos.vel);
    return vel_limited;
}

float Motion1DPositionVelocitySingleTimed::get_time_remaining_for(motion_state_s* current_state, motion_state_s* target_state, float arrival_time) {
    float pos_distance = abs(current_state->pos - target_state->pos);
    float vel = pos_distance > 0 ? limit_pos.vel : -limit_neg.vel;
    return max(arrival_time, pos_distance / vel);
}

bool Motion1DPositionVelocitySingleTimed::check_input_and_config_errors() {

    bool error_found = false;

    error_code.bits.invalid_dt = dt == 0 || isnan(dt);
    if (error_code.bits.invalid_dt) error_found = true;

    error_code.bits.limit_pos_nan = isnan(limit_neg.pos) || isnan(limit_pos.pos);
    if (error_code.bits.limit_pos_nan) error_found = true;

    error_code.bits.limit_vel_nan = isnan(limit_neg.vel) || isnan(limit_pos.vel);
    if (error_code.bits.limit_vel_nan) error_found = true;

    error_code.bits.limit_pos_polarity = !isnan(limit_neg.pos) && !isnan(limit_pos.pos) && limit_neg.pos >= limit_pos.pos;
    if (error_code.bits.limit_pos_polarity) error_found = true;

    error_code.bits.limit_vel_polarity = !isnan(limit_neg.vel) && !isnan(limit_pos.vel) && limit_neg.vel >= limit_pos.vel;
    if (error_code.bits.limit_vel_polarity) error_found = true;

    error_code.bits.tgt_acc_not_impl = !isnan(user.acc) && isnan(user.pos);
    if (error_code.bits.tgt_acc_not_impl) error_found = true;

    error_code.bits.no_tgt = isnan(user.pos) && isnan(user.vel);
    if (error_code.bits.no_tgt) error_found = true;

    return !error_found;
}

bool Motion1DPositionVelocitySingleTimed::check_output_errors() {

    bool error_found = false;

    error_code.bits.cur_pos_nan = isnan(cur.pos);
    if (error_code.bits.cur_pos_nan) error_found = true;

    error_code.bits.cur_vel_nan = isnan(cur.vel);
    if (error_code.bits.cur_vel_nan) error_found = true;

    error_code.bits.cur_acc_nan = isnan(cur.acc);
    if (error_code.bits.cur_acc_nan) error_found = true;

    error_code.bits.cur_jrk_nan = isnan(cur.jrk);
    if (error_code.bits.cur_jrk_nan) error_found = true;

    return !error_found;
}
