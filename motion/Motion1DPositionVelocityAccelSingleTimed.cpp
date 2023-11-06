#include "Motion1DPositionVelocityAccelSingleTimed.h"
#include <cstring>

using namespace std;

// note: multiple targets can be set at once.
// when this happens, targetting is controlled by the highest-order parameter.
// lower-order parameters are interpreted as the target values for those orders at the instant the higher-order target finishes.
// so vel=10, acc=2 means:
//  - seek vel 10
//  - arrive at vel 10 with an acc of 2
//  - continue with a constant acc of 2 forever (leaving vel 10)
//  - this may hit a vel limit, where it will decel to constant vel at the limit vel

void Motion1DPositionVelocityAccelSingleTimed::configure_position_limits(float _limit_pos_neg, float _limit_pos_pos) {
    configure_position_limits_int(_limit_pos_neg, _limit_pos_pos);
}

void Motion1DPositionVelocityAccelSingleTimed::configure_velocity_limits(float _limit_vel_neg, float _limit_vel_pos) {
    configure_velocity_limits_int(_limit_vel_neg, _limit_vel_pos);
    // note: we are deliberately misaligning the motion orders here.
    // see get_jerk_for_accel_mode() and get_jerk_for_velocity_mode() for how these are used.
    // motionpva is the same order.  it's for when we dont have a time target
    motion_pv.configure_position_limits(_limit_vel_neg, _limit_vel_pos);
    motion_pvt.configure_position_limits(_limit_vel_neg, _limit_vel_pos);
}

void Motion1DPositionVelocityAccelSingleTimed::configure_acceleration_limits(float _limit_acc_neg, float _limit_acc_pos) {
    configure_acceleration_limits_int(_limit_acc_neg, _limit_acc_pos);
    // note: we are deliberately misaligning the motion orders here.
    // see get_jerk_for_accel_mode() and get_jerk_for_velocity_mode() for how these are used.
    // motionpva is the same order.  it's for when we dont have a time target
    motion_pv.configure_velocity_limits(_limit_acc_neg, _limit_acc_pos);
    motion_pvt.configure_velocity_limits(_limit_acc_neg, _limit_acc_pos);
}

void Motion1DPositionVelocityAccelSingleTimed::configure_jerk_limits(float _limit_jrk) {
    configure_jerk_limits_int(_limit_jrk);
}

void Motion1DPositionVelocityAccelSingleTimed::configure_dt(float _dt) {
    configure_dt_int(_dt);
    motion_pv.configure_dt(_dt);
    motion_pvt.configure_dt(_dt);
    motion_pva.configure_dt(_dt);
}

void Motion1DPositionVelocityAccelSingleTimed::reset_state() {
    // since there's no constructor / init function, the initial values of these variables must match the below values
    using_position_limit = false;
    position_limit = 0;
    arrived_at_position_target = false;
    arrived_at_position_target_now = false;

    motion_pv.reset_state();
    motion_pvt.reset_state();
    motion_pva.reset_state();
}

void Motion1DPositionVelocityAccelSingleTimed::run_next_timestep() {

    // if there are any configuration or user input errors, report them and exit immediately.
    // these must be corrected at design-time by fixing the calling code.

    if (!check_input_and_config_errors()) return;

    if (target_has_changed_since_last_run(&user)) { // todo: needs to have a seperate var for user-supplied target vs. internal count-down
        reset_state();
    }

    // store last state temporary so that they can be used to calculate the new values

    last = cur;

    // figure out which dimension of motion we're targeting - velocity or acceleration

    determine_target_mode();

    // run the relevant controller to come up with the new jerk value, which will determine the next state of motion

    float accel = NAN;
    float computed_time_remaining = 0;

    switch (target_mode) {

        case target_mode_e::velocity:
            accel = get_accel_for_velocity_mode(&last, &target, user_arrival_time, &computed_time_remaining);
            cur = compute_next_state_from_accel(&last, accel, dt);
            break;

        case target_mode_e::position:
            cur = get_accel_for_position_mode(&last, &target, user_arrival_time, &computed_time_remaining);
            if (computed_time_remaining < dt && !arrived_at_position_target) {
                arrived_at_position_target = true;
                arrived_at_position_target_now = true;
            }
            break;

        default:
            break;

    }

    cur_time_remaining = using_position_limit ? NAN : (arrived_at_position_target ? 0 : computed_time_remaining);

    // if we just hit the velocity target this frame, adjust our velocity and position so that they exactly match the targets.
    // this is designed to correct for floating-point math errors only - anything bigger than that means that the equations are wrong and should be fixed!

    if (arrived_at_position_target_now) {
        arrived_at_position_target_now = false;
        cur.vel = isnan(target.vel) ? 0 : target.vel;
        cur.pos = target.pos;
    }

    // decrement the user's input time remaining

    user_arrival_time = max(0.0f, user_arrival_time - dt);

    // check for errors in the output signals

    check_output_errors();
}

float Motion1DPositionVelocityAccelSingleTimed::get_accel_for_velocity_mode(motion_state_s* current_state, motion_state_s* target_state, float arrival_time_target, float* time_remaining) {

    // we use the 2nd-order module for this.
    // since it works in position/velocity and we have jerk/accel, everything has to be shifted by two orders

    float acc = 0;

    if (user_arrival_time > 0) {
        motion_pvt.jump_to(current_state->vel);
        motion_pvt.set_target(target_state->vel);
        motion_pvt.set_arrival_time(user_arrival_time);
        motion_pvt.run_next_timestep();
        acc = motion_pvt.get_velocity();
        *time_remaining = motion_pvt.get_time_remaining();
    }

    else {
        motion_pv.jump_to(current_state->vel);
        motion_pv.set_target(target_state->vel);
        motion_pv.run_next_timestep();
        acc = motion_pv.get_velocity();
        *time_remaining = motion_pv.get_time_remaining();
    }

    return acc;
}

Motion1DInterfaceSingle::motion_state_s Motion1DPositionVelocityAccelSingleTimed::get_accel_for_position_mode(motion_state_s* current_state, motion_state_s* target_state, float arrival_time_target, float* time_remaining) {

    motion_state_s next_state = {0, 0, 0, 0};

    int best_solver = 0;

    if (user_arrival_time > 0) {

        float t[8] = {0}, a[8] = {0};
        float best_t[8] = {0}, best_a[8] = {0};
        best_t[3] = INFINITY;

        // try all of the different possible curves that could solve the motion problem, in order by least segments to most

        if (solve_position_with_velocity_limit(current_state, target_state, arrival_time_target, limit_pos.vel, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, 3);
        if (solve_position_with_velocity_limit(current_state, target_state, arrival_time_target, limit_neg.vel, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, -3);
        if (solve_position_no_velocity_limit_2_phase(current_state, target_state, arrival_time_target, 0, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, 2);
        if (solve_position_no_velocity_limit_1_phase(current_state, target_state, arrival_time_target, 0, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, 1);
        if (solve_position_no_velocity_limit_1_phase_drift(current_state, target_state, arrival_time_target, 0, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, 4);

        // store the solution for use by get_state_at_time()

        last_solution_var = 'a';
        memcpy(last_solution_t, best_t, sizeof(best_t));
        memcpy(last_solution_a, best_a, sizeof(best_a));
        memcpy(&last_ics, current_state, sizeof(last_ics));

        // next state determination

        next_state = get_state_at_time(dt);

        *time_remaining = best_t[3];

    }

    if (user_arrival_time == 0 || best_solver == 0) {

        motion_pva.configure_position_limits(limit_neg.pos, limit_pos.pos);
        motion_pva.configure_velocity_limits(limit_neg.vel, limit_pos.vel);
        motion_pva.configure_acceleration_limits(limit_neg.acc, limit_pos.acc);
        motion_pva.jump_to(current_state->pos, current_state->vel, current_state->acc);
        motion_pva.set_target(target_state->pos, target_state->vel, target_state->acc);
        motion_pva.run_next_timestep();
        *time_remaining = motion_pva.get_time_remaining();

        last_solution_var = 'a';
        float t[8] = {0}, a[8] = {0}, j[8] = {0};
        motion_pva.get_last_solution(t, a, j);
        memcpy(last_solution_t, t, sizeof(t));
        memcpy(last_solution_a, a, sizeof(a));
        memcpy(&last_ics, current_state, sizeof(last_ics));

        next_state = get_state_at_time(dt);
    }

    return next_state;

}

void Motion1DPositionVelocityAccelSingleTimed::update_best_solver(float t[], float a[], float best_t[], float best_a[], int* best_solver, int this_solver) {

    // this is a helper method for get_state_for_position_mode().
    // it determines if the current solution is better than the previous best solution, and updates data accordingly.
    // if the new finishing time is quicker it becomes the best.
    // if the new finishing time ties with the best (with some margin for floating-point error), then the solution with fewer stages wins.

    bool new_is_best = false;
    int last_used_index = 3;

    if (t[last_used_index] < best_t[last_used_index] - 0.00001) {
        new_is_best = true;
    }

    else if (abs(t[last_used_index] - best_t[last_used_index]) < 0.00001) {
        if (abs(t[last_used_index] - best_t[last_used_index]) < 1e-6) {
            if (abs(this_solver) < abs(*best_solver)) new_is_best = true;
        }
    }

    if (new_is_best) {
        memcpy(best_t, t, 8 * sizeof(float));
        memcpy(best_a, a, 8 * sizeof(float));
        *best_solver = this_solver;
    }
}

bool Motion1DPositionVelocityAccelSingleTimed::solve_position_with_velocity_limit(motion_state_s* current_state, motion_state_s* target_state, float target_time_remaining, float vmax, float t_out[], float a_out[]) {

    // solve motion model

    float v0 = current_state->vel;
    float p0 = current_state->pos;
    float v3 = isnan(target_state->vel) ? 0 : target_state->vel; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
    float p3 = target_state->pos;
    float t3 = user_arrival_time;
    float v1 = vmax;
    float v2 = vmax;

    float a2 = -(v0*v0 - 2*v0*v1 + v1*v1 + 2*v1*v2 - 2*v1*v3 - v2*v2 + v3*v3) / (2*p0 - 2*p3 + 2*t3*v1);
    float a0 = -a2;
    float t1 = (v0 - v1) / a2;
    float t2 = (v2 - v3 + a2*t3) / a2;
    float p1 = -0.5*a2*t1*t1 + v0*t1 + p0;
    float p2 = p1 - v1*(t1-t2);

    // check to see if the result is valid

    bool valid = true;
    if (a0 > 0) {
        if (a0 > limit_pos.acc) valid = false;
        if (a2 < limit_neg.acc) valid =false;
    }
    else {
        if (a0 < limit_neg.acc) valid = false;
        if (a2 > limit_pos.acc) valid = false;
    }
    if (t1 < 0) valid = false;
    if (t2 < 0) valid = false;
    if (t1 > t3) valid = false;
    if (t2 > t3) valid = false;
    // todo: probably also need some srt of projection test

    // if result is valid, output the path

    if (valid) {
        t_out[1] = t1;
        t_out[2] = t2;
        t_out[3] = t3;
        a_out[0] = a0;
        a_out[1] = 0;
        a_out[2] = a2;

        return true;
    }

    else {
        return false;
    }

}

bool Motion1DPositionVelocityAccelSingleTimed::solve_position_no_velocity_limit_2_phase(motion_state_s* current_state, motion_state_s* target_state, float target_time_remaining, float vlimit, float t_out[], float a_out[]) {

    // solve motion model

    float v0 = current_state->vel;
    float p0 = current_state->pos;
    float v2 = isnan(target_state->vel) ? 0 : target_state->vel; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
    float p2 = target_state->pos;
    float t2 = target_time_remaining;

    // make t3 not be zero to avoid divide-by-zero, but still allow it to be less than dt so that it can trigger the arrival logic
    t2 = max(0.5f * dt, t2);

    float x1 = sqrtf(2) * sqrtf(2*p0*p0 - 4*p0*p2 + 2*p0*t2*v0 + 2*p0*t2*v2 + 2*p2*p2 - 2*p2*t2*v0 - 2*p2*t2*v2 + t2*t2*v0*v0 + t2*t2*v2*v2);
    float a0a = -(2*p0 - 2*p2 + x1 + t2*v0 + t2*v2) / (t2*t2);
    float a0b = -(2*p0 - 2*p2 - x1 + t2*v0 + t2*v2) / (t2*t2);
    float a0 = NAN;
    if (!isnanf(a0a) && (isnanf(a0b) || fabs(a0a) > fabs(a0b))) a0 = a0a;
    if (!isnanf(a0b) && (isnanf(a0a) || fabs(a0b) > fabs(a0a))) a0 = a0b;
    float t1 = (v2 - v0 + a0*t2) / (2*a0);
    float v1 = v0 + a0*t1;
    float a1 = -a0;

    // check to see if the result is valid

    bool valid = true;
    uint8_t reason = 0;

    if (v1 < limit_neg.vel * 1.001 || v1 > limit_pos.vel * 1.001) {
        reason += 1;
        valid = false;
    }
    if (a0 > 0) {
        if (a0 > limit_pos.acc) {
            reason += 2;
            valid = false;
        }
        if (a1 < limit_neg.acc) {
            reason += 4;
            valid =false;
        }
    }
    else {
        if (a0 < limit_neg.acc) {
            reason += 8;
            valid = false;
        }
        if (a1 > limit_pos.acc) {
            reason += 16;
            valid = false;
        }
    }
    if (t1 < -0.01f) {
        reason += 32;
        valid = false;
    }
    if (t1 > t2) {
        reason += 64;
        valid = false;
    }
    if (isnanf(a0) || isnanf(a1) || isnanf(t1) || isnanf(t2)) {
    	valid = false;
    	reason += 128;
    }

    // if result is valid, output the path

    if (valid) {
        t_out[1] = 0;
        a_out[1] = a0;
        t_out[2] = t1;
        a_out[2] = a1;
        t_out[3] = t2;
        a_out[3] = 0;

        return true;
    }

    else {
        return false;
    }

}

bool Motion1DPositionVelocityAccelSingleTimed::solve_position_no_velocity_limit_1_phase(motion_state_s* current_state, motion_state_s* target_state, float target_time_remaining, float vlimit, float t_out[], float a_out[]) {

    // solve motion model

    float v0 = current_state->vel;
    float p0 = current_state->pos;
    float v2 = isnan(target_state->vel) ? 0 : target_state->vel; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
    float p2 = target_state->pos;
    float t2 = target_time_remaining;
    float t1 = 0;
    float v1 = v0;
    float p1 = p0;

    // make t3 not be zero to avoid divide-by-zero, but still allow it to be less than dt so that it can trigger the arrival logic
    t2 = max(0.5f * dt, t2);

    float a1_from_ev2 = (v2-v1)/t2;
    float a1_from_ep2 = (-2*(p1-p2+t2*v1))/(t2*t2);
    float a1 = a1_from_ep2;

    // check to see if the result is valid

    bool valid = true;
    uint8_t reason = 0;

    if (fabs(a1_from_ep2 - a1_from_ev2) > 0.0001f) {
        valid = false;
        reason += 1;
    }

    if (a1 < 0) {
        if (a1 < limit_neg.acc) {
            reason += 4;
            valid =false;
        }
    }
    else {
        if (a1 > limit_pos.acc) {
            reason += 16;
            valid = false;
        }
    }
    if (t2 < -0.0001f) {
        reason += 32;
        valid = false;
    }

    // if result is valid, output the path

    if (valid) {
        t_out[1] = 0;
        a_out[1] = 0;
        t_out[2] = t1;
        a_out[2] = a1;
        t_out[3] = t2;
        a_out[3] = 0;

        return true;
    }

    else {
        return false;
    }

}

bool Motion1DPositionVelocityAccelSingleTimed::solve_position_no_velocity_limit_1_phase_drift(motion_state_s* current_state, motion_state_s* target_state, float target_time_remaining, float vlimit, float t_out[], float a_out[]) {

    // solve motion model

    float v0 = current_state->vel;
    float p0 = current_state->pos;
    float v2 = isnan(target_state->vel) ? 0 : target_state->vel; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
    float p2 = target_state->pos;
    float t2 = target_time_remaining;
    float a0 = 0;

    float a1 = -(v0*v0 - 2*v0*v2 + v2*v2) / (2*p0 - 2*p2 + 2*t2*v0);
    float t1 = (v0 - v2 + a1*t2) / a1;
    float p1 = p0 + v0*t1 + 0.5*a0*t1*t1;

    // check to see if the result is valid

    bool valid = true;
    uint8_t reason = 0;

    if (isnan(a1) || isnan(t1)) {
        reason += 1;
        valid = false;
    }
    if (t1 < -0.000001f) {
        reason += 2;
        valid = false;
    }
    if (t1 > t2 + 0.000001f) {
        reason += 4;
        valid = false;
    }
    if (a1 < limit_neg.acc || a1 > limit_pos.acc) {
        reason += 8;
        valid = false;
    }

    // if result is valid, output the path

    if (valid) {
        t_out[1] = 0;
        a_out[1] = 0;
        t_out[2] = t1;
        a_out[2] = a1;
        t_out[3] = t2;
        a_out[3] = 0;

        return true;
    }

    else {
        return false;
    }

}

bool Motion1DPositionVelocityAccelSingleTimed::target_has_changed_since_last_run(motion_state_s* target_state) {
    bool pos_ok = (isnan(target_state->pos) && isnan(last_user_target.pos)) || (target_state->pos == last_user_target.pos);
    bool vel_ok = (isnan(target_state->vel) && isnan(last_user_target.vel)) || (target_state->vel == last_user_target.vel);

    if (!pos_ok || !vel_ok) {
        last_user_target = *target_state;
        return true;
    }
    else {
        return false;
    }
}

void Motion1DPositionVelocityAccelSingleTimed::determine_target_mode() {

    // decide if we're in position or velocity target mode
    // this is based on:
    //  - the user's input
    //  - if we had a position target but reached it and switched to velocity mode
    //  - if we're going to exceed a position limit and need to switch to position mode to respect that limit

    target_mode = none;
    target.pos = NAN;
    target.vel = NAN;
    target.acc = NAN;

    // start by assuming the user's specified target mode

    if (!isnan(user.pos)) {
        target_mode = position;
        target.pos = user.pos;
        target.vel = user.vel;
    }

    else if (!isnan(user.vel)) {
        target_mode = velocity;
        target.vel = user.vel;
    }

    // if we're in position mode, but have arrived at the target, switch to velocity mode to honor the final velocity target.
    // note this is stateful behavior and reset each time new targets are set.
    // if the user is operating in this mode, they cannot send a continuous stream of target info.

    if (arrived_at_position_target) {
        target_mode = velocity;
        target.vel = user.vel;
    }

    // if we're in velocity mode but going to overrun a position limit, switch to position mode at the limit

//    motion_state_s next = compute_next_state_from_accel(&cur, cur.acc, dt);
//    float stopping_acc = next.vel > 0 ? limit_neg.acc : limit_pos.acc;
//    float time_to_make_vel_zero = abs(next.vel / stopping_acc);
//    float position_if_stopped_now = next.pos + next.vel * time_to_make_vel_zero + 0.5 * stopping_acc * powf(time_to_make_vel_zero, 2);
//
//    bool position_in_bounds = cur.pos > limit_neg.pos && cur.pos < limit_pos.pos;
//
//    if (!using_position_limit && position_in_bounds && target_mode == velocity) {
//        if (position_if_stopped_now > limit_pos.pos) {
//            using_position_limit = true;
//            position_limit = limit_pos.pos;
//            arrived_at_position_target = false;
//        }
//        if (position_if_stopped_now < limit_neg.pos) {
//            using_position_limit = true;
//            position_limit = limit_neg.pos;
//            arrived_at_position_target = false;
//        }
//    }
//
//    if (using_position_limit) {
//        target_mode = position;
//        target.pos = position_limit;
//        target.vel = 0;
//    }

}

bool Motion1DPositionVelocityAccelSingleTimed::check_input_and_config_errors() {

    bool error_found = false;

    error_code.bits.invalid_dt = dt == 0 || isnan(dt);
    if (error_code.bits.invalid_dt) error_found = true;

    error_code.bits.limit_acc_nan = isnan(limit_neg.acc) || isnan(limit_pos.acc);
    if (error_code.bits.limit_acc_nan) error_found = true;

    error_code.bits.limit_vel_nan = isnan(limit_neg.vel) || isnan(limit_pos.vel);
    if (error_code.bits.limit_vel_nan) error_found = true;

    error_code.bits.limit_pos_nan = isnan(limit_neg.pos) || isnan(limit_pos.pos);
    if (error_code.bits.limit_pos_nan) error_found = true;

    error_code.bits.limit_acc_polarity = !isnan(limit_neg.acc) && !isnan(limit_pos.acc) && limit_neg.acc > limit_pos.acc;
    if (error_code.bits.limit_acc_polarity) error_found = true;

    error_code.bits.limit_vel_polarity = !isnan(limit_neg.vel) && !isnan(limit_pos.vel) && limit_neg.vel > limit_pos.vel;
    if (error_code.bits.limit_vel_polarity) error_found = true;

    error_code.bits.limit_pos_polarity = !isnan(limit_neg.pos) && !isnan(limit_pos.pos) && limit_neg.pos >= limit_pos.pos;
    if (error_code.bits.limit_pos_polarity) error_found = true;

    error_code.bits.no_tgt = isnan(user.pos) && isnan(user.vel);
    if (error_code.bits.no_tgt) error_found = true;

    return !error_found;
}

bool Motion1DPositionVelocityAccelSingleTimed::check_output_errors() {

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

