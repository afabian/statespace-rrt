#include "Motion1DPositionVelocityAccelDouble.h"
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

void Motion1DPositionVelocityAccelDouble::configure_position_limits(double _limit_pos_neg, double _limit_pos_pos) {
    configure_position_limits_int(_limit_pos_neg, _limit_pos_pos);
}

void Motion1DPositionVelocityAccelDouble::configure_velocity_limits(double _limit_vel_neg, double _limit_vel_pos) {
    configure_velocity_limits_int(_limit_vel_neg, _limit_vel_pos);
    // note: we are deliberately misaligning the motion orders here.
    // see get_jerk_for_accel_mode() and get_jerk_for_velocity_mode() for how these are used.
    motion_pv.configure_position_limits(_limit_vel_neg, _limit_vel_pos);
}

void Motion1DPositionVelocityAccelDouble::configure_acceleration_limits(double _limit_acc_neg, double _limit_acc_pos) {
    configure_acceleration_limits_int(_limit_acc_neg, _limit_acc_pos);
    // note: we are deliberately misaligning the motion orders here.
    // see get_jerk_for_accel_mode() and get_jerk_for_velocity_mode() for how these are used.
    motion_pv.configure_velocity_limits(_limit_acc_neg, _limit_acc_pos);
}

void Motion1DPositionVelocityAccelDouble::configure_jerk_limits(double _limit_jrk) {
    configure_jerk_limits_int(_limit_jrk);
}

void Motion1DPositionVelocityAccelDouble::configure_dt(double _dt) {
    configure_dt_int(_dt);
    motion_pv.configure_dt(_dt);
}

void Motion1DPositionVelocityAccelDouble::reset_state() {
    // since there's no constructor / init function, the initial values of these variables must match the below values
    using_position_limit = false;
    position_limit = 0;
    arrived_at_position_target = false;
    arrived_at_position_target_now = false;

    motion_pv.reset_state();
}

void Motion1DPositionVelocityAccelDouble::run_next_timestep() {

    // if there are any configuration or user input errors, report them and exit immediately.
    // these must be corrected at design-time by fixing the calling code.

    if (!check_input_and_config_errors()) return;

    if (target_has_changed_since_last_run(&user)) {
        reset_state();
    }

    // store last state temporary so that they can be used to calculate the new values

    last = cur;

    // figure out which dimension of motion we're targeting - velocity or acceleration

    determine_target_mode();

    // run the relevant controller to come up with the new jerk value, which will determine the next state of motion

    double accel = NAN;
    double computed_time_remaining = 0;

    switch (target_mode) {

        case target_mode_e::velocity:
            accel = get_accel_for_velocity_mode(&last, &target, &computed_time_remaining);
            cur = compute_next_state_from_accel(&last, accel, dt);
            break;

        case target_mode_e::position:
            cur = get_accel_for_position_mode(&last, &target, &computed_time_remaining);
            if (computed_time_remaining < dt && !arrived_at_position_target) {
                arrived_at_position_target = true;
                arrived_at_position_target_now = true;
            }
            break;

        default:
            break;

    }

    cur_time_remaining = (using_position_limit || using_velocity_limit) ? INFINITY : (arrived_at_position_target ? 0 : computed_time_remaining);

    // if we just hit the velocity target this frame, adjust our velocity and acceleration so that they exactly match the targets.
    // this is designed to correct for doubleing-point math errors only - anything bigger than that means that the equations are wrong and should be fixed!

    if (arrived_at_position_target_now) {
        arrived_at_position_target_now = false;
        cur.vel = isnan(target.vel) ? 0 : target.vel;
        cur.pos = target.pos;
    }

    // check for errors in the output signals

    check_output_errors();
}

double Motion1DPositionVelocityAccelDouble::get_accel_for_velocity_mode(motion_state_s* current_state, motion_state_s* target_state, double* time_remaining) {

    // we use the 2nd-order module for this.
    // since it works in position/velocity and we have vel/accel, everything has to be shifted by one order
    motion_pv.jump_to(isnan(current_state->vel) ? 0 : current_state->vel);
    motion_pv.set_target(isnan(target_state->vel) ? 0 : target_state->vel);
    motion_pv.reset_state();
    motion_pv.run_next_timestep();
    double acc = motion_pv.get_velocity();
    *time_remaining = motion_pv.get_time_remaining();

    best_solver = -9;

    return acc;
}

Motion1DInterfaceDouble::motion_state_s Motion1DPositionVelocityAccelDouble::get_accel_for_position_mode(motion_state_s* current_state, motion_state_s* target_state, double* time_remaining) {

    double t[8] = {0}, a[8] = {0};
    double best_t[8] = {0}, best_a[8] = {0};
    best_solver = 0;
    best_t[3] = INFINITY;

    // try all of the different possible curves that could solve the motion problem, in order by least segments to most

    if (solve_position_with_velocity_limit(current_state, target_state, limit_pos.acc, limit_neg.acc, limit_pos.vel, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, 4);
    if (solve_position_with_velocity_limit(current_state, target_state, limit_neg.acc, limit_pos.acc, limit_neg.vel, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, -4);
    if (solve_position_with_velocity_limit_from_outofbounds(current_state, target_state, limit_pos.acc, limit_neg.acc, limit_pos.vel, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, 5);
    if (solve_position_with_velocity_limit_from_outofbounds(current_state, target_state, limit_neg.acc, limit_pos.acc, limit_neg.vel, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, -5);
    if (solve_position_no_velocity_limit_2_phase(current_state, target_state, limit_pos.acc, limit_neg.acc, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, 3);
    if (solve_position_no_velocity_limit_2_phase(current_state, target_state, limit_neg.acc, limit_pos.acc, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, -3);
    if (solve_position_no_velocity_limit_1_phase(current_state, target_state, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, 2);
    if (solve_position_no_velocity_limit_1_phase_drift(current_state, target_state, limit_neg.acc, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, 1);
    if (solve_position_no_velocity_limit_1_phase_drift(current_state, target_state, limit_pos.acc, t, a)) update_best_solver(t, a, best_t, best_a, &best_solver, -1);

    // store the solution for use by get_state_at_time()

    last_solution_var = 'a';
    memcpy(last_solution_t, best_t, sizeof(best_t));
    memcpy(last_solution_a, best_a, sizeof(best_a));
    memcpy(&last_ics, current_state, sizeof(last_ics));

    // next state determination

    motion_state_s next_state = get_state_at_time(dt);

    *time_remaining = best_t[3];

    debug[5] = best_solver;

    return next_state;

}

void Motion1DPositionVelocityAccelDouble::update_best_solver(double t[], double a[], double best_t[], double best_a[], int* best_solver, int this_solver) {

    // this is a helper method for get_state_for_position_mode().
    // it determines if the current solution is better than the previous best solution, and updates data accordingly.
    // if the new finishing time is quicker it becomes the best.
    // if the new finishing time ties with the best (with some margin for doubleing-point error), then the solution with fewer stages wins.

    bool new_is_best = false;
    int last_used_index = 3;

    if (abs(t[last_used_index] - best_t[last_used_index]) < 1e-6) {
        int phases_old = 0;
        int phases_new = 0;
        for (int i=0; i<=last_used_index; i++) {
            if (t[i] != 0) phases_new++;
            if (best_t[i] != 0) phases_old++;
        }
        if (phases_new < phases_old) new_is_best = true;
    }

    else if (t[last_used_index] < best_t[last_used_index]) {
        new_is_best = true;
    }

    if (new_is_best) {
        memcpy(best_t, t, 8 * sizeof(double));
        memcpy(best_a, a, 8 * sizeof(double));
        *best_solver = this_solver;
    }
}

bool Motion1DPositionVelocityAccelDouble::solve_position_with_velocity_limit(motion_state_s* current_state, motion_state_s* target_state, double ap, double an, double vmax, double t_out[], double a_out[]) {

    // solve motion model

    double v0 = current_state->vel;
    double p0 = current_state->pos;
    double a0 = ap;
    double v1 = vmax;
    double a1 = 0;
    double v2 = vmax;
    double a2 = an;
    double v3 = isnan(target_state->vel) ? 0 : target_state->vel; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
    double p3 = target_state->pos;

    double t2 = -(p0 - p3 + pow(v1-v3,2)/(2*an) + pow(v0-v1,2)/(2*ap) - (v2*(v1-v3))/an - (v0*(v0-v1))/ap + (v1*(v0-v1))/ap) / (v1);
    double t3 = (v3 - v2 + an*t2) / an;
    double t1 = -(v0 - v1) / ap;

    double t12 = t2 - t1;
    double t23 = t3 - t2;

    // check to see if the result is valid

    double v1_from_v0 = v0 + t1*a0;
    double v2_from_v0 = v1_from_v0 + t12*a1;
    double v3_from_v0 = v2_from_v0 + t23*a2;

    double p1_from_p0 = p0 + t1*v0 + 0.5*a0*t1*t1;
    double p2_from_p0 = p1_from_p0 + t12*v1 + 0.5*a1*t12*t12;
    double p3_from_p0 = p2_from_p0 + t23*v2 + 0.5*a2*t23*t23;

    bool valid = true;
    uint8_t reason = 0;


    if (t1 < -0.00001f) {
        valid = false;
        reason += 1;
    }

    if (t12 < -0.00001f) {
        valid = false;
        reason += 2;
    }

    if (t23 < -0.00001f) {
        valid = false;
        reason += 4;
    }

    if (fabs(v3_from_v0 - v3) > v_threshold) {
        valid = false;
        reason += 8;
    }

    if (fabs(p3_from_p0 - p3) > p_threshold) {
        valid = false;
        reason += 16;
    }

    // if result is valid, output the path

    if (valid) {
        t_out[0] = 0;
        a_out[0] = a0;

        t_out[1] = t1;
        a_out[1] = 0;

        t_out[2] = t2;
        a_out[2] = a2;

        t_out[3] = t3;
        a_out[3] = 0;

        return true;
    }

    else {
        return false;
    }
}

bool Motion1DPositionVelocityAccelDouble::solve_position_with_velocity_limit_from_outofbounds(motion_state_s* current_state, motion_state_s* target_state, double ap, double an, double vmax, double t_out[], double a_out[]) {

    // solve motion model

    double v0 = current_state->vel;
    double p0 = current_state->pos;
    double a0 = an;
    double v1 = vmax;
    double a1 = 0;
    double v2 = vmax;
    double a2 = an;
    double v3 = isnan(target_state->vel) ? 0 : target_state->vel; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
    double p3 = target_state->pos;

    double t2 = -(p0 - p3 + pow(v0-v1,2)/(2*an) + pow(v1-v3,2)/(2*an) - (v0*(v0-v1))/an + (v1*(v0-v1))/an - (v2*(v1-v3))/an) / (v1);
    double t3 = (v3 - v2 + an*t2) / an;
    double t1 = -(v0 - v1) / an;

    double t12 = t2 - t1;
    double t23 = t3 - t2;

    // check to see if the result is valid

    double v1_from_v0 = v0 + t1*a0;
    double v2_from_v0 = v1_from_v0 + t12*a1;
    double v3_from_v0 = v2_from_v0 + t23*a2;

    double p1_from_p0 = p0 + t1*v0 + 0.5*a0*t1*t1;
    double p2_from_p0 = p1_from_p0 + t12*v1 + 0.5*a1*t12*t12;
    double p3_from_p0 = p2_from_p0 + t23*v2 + 0.5*a2*t23*t23;

    bool valid = true;
    uint8_t reason = 0;


    if (t1 < -0.00001f) {
        valid = false;
        reason += 1;
    }

    if (t12 < -0.00001f) {
        valid = false;
        reason += 2;
    }

    if (t23 < -0.00001f) {
        valid = false;
        reason += 4;
    }

    if (fabs(v3_from_v0 - v3) > v_threshold) {
        valid = false;
        reason += 8;
    }

    if (fabs(p3_from_p0 - p3) > p_threshold) {
        valid = false;
        reason += 16;
    }

    // if result is valid, output the path

    if (valid) {
        t_out[0] = 0;
        a_out[0] = a0;

        t_out[1] = t1;
        a_out[1] = 0;

        t_out[2] = t2;
        a_out[2] = a2;

        t_out[3] = t3;
        a_out[3] = 0;

        return true;
    }

    else {
        return false;
    }
}

bool Motion1DPositionVelocityAccelDouble::solve_position_no_velocity_limit_2_phase(motion_state_s* current_state, motion_state_s* target_state, double ap, double an, double t_out[], double a_out[]) {

    // solve motion model

    double v0 = current_state->vel;
    double p0 = current_state->pos;
    double v2 = isnan(target_state->vel) ? 0 : target_state->vel; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
    double p2 = target_state->pos;

    double x1 = sqrt((an-ap) * (an*v0*v0 - ap*v2*v2 - 2*an*ap*p0 + 2*an*ap*p2));
    double t1a =  (ap*v0 - an*v0 + x1) / (an*ap - ap*ap);
    double t1b = -(an*v0 - ap*v0 + x1) / (an*ap - ap*ap);
    double t1 = max(t1a, t1b);
    double v1 = v0 + ap*t1;
    double p1 = p0 + v0*t1 + 0.5*ap*t1*t1;
    double t2 = (v2 - v1 + an*t1) / an;

    // check to see if the result is valid

    bool valid = true;
    uint8_t reason = 0;

    double t12 = t2 - t1;
    double v2_from_v1 = v1 + an*t12;
    double p2_from_p1 = p1 + v1*t12 + 0.5*an*t12*t12;

    if (fabs(p2_from_p1 - p2) > v_threshold) {
        valid = false;
        reason += 1;
    }

    if (fabs(v2_from_v1 - v2) > p_threshold) {
        valid = false;
        reason += 2;
    }

    if (t1 < -0.00001f) {
        valid = false;
        reason += 4;
    }

    if (t2 - t1 < -0.00001f) {
        valid = false;
        reason += 8;
    }

    if (v1 > limit_pos.vel * 1.0001f || v1 < limit_neg.vel * 1.0001f) {
        valid = false;
        reason = 16;
    }

    if (isnan(t1) || isnan(t2)) {
        valid = false;
        reason = 32;
    }

    // if result is valid, output the path

    if (valid) {
        t_out[1] = 0;
        a_out[1] = ap;
        t_out[2] = t1;
        a_out[2] = an;
        t_out[3] = t2;
        a_out[3] = 0;

        return true;
    }

    else {
        return false;
    }
}

bool Motion1DPositionVelocityAccelDouble::solve_position_no_velocity_limit_1_phase(motion_state_s* current_state, motion_state_s* target_state, double t_out[], double a_out[]) {

    // solve motion model

    double v0 = current_state->vel;
    double p0 = current_state->pos;
    double v2 = isnan(target_state->vel) ? 0 : target_state->vel; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
    double p2 = target_state->pos;
    double t1 = 0;
    double v1 = v0;
    double p1 = p0;

    double a1 = v2 - v1 > 0 ? limit_pos.acc : limit_neg.acc;

    double t2 = (v2 - v1) / a1;

    // check to see if the result is valid

    bool valid = true;
    uint8_t reason = 0;

    double p2_from_p1 = p1 + v1*t2 + 0.5*a1*t2*t2;

    if (fabs(p2_from_p1 - p2) > p_threshold) {
        valid = false;
        reason += 1;
    }

    if (t2 < -0.00001f) {
        valid = false;
        reason += 2;
    }

    if (isnan(t1) || isnan(t2)) {
        valid = false;
        reason = 4;
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

bool Motion1DPositionVelocityAccelDouble::solve_position_no_velocity_limit_1_phase_drift(motion_state_s* current_state, motion_state_s* target_state, double an, double t_out[], double a_out[]) {

    // solve motion model

    double a0 = 0;
    double v0 = current_state->vel;
    double p0 = current_state->pos;
    double a1 = an;
    double v1 = v0;
    double v2 = isnan(target_state->vel) ? 0 : target_state->vel; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
    double p2 = target_state->pos;

    double t2 = -(p0 - p2 + pow(v1-v2,2)/(2*an) + (v0*(v1-v2))/an - (v1*(v1-v2))/an) / v0;
    double t1 = (v1 - v2 + an*t2) / an;

    // check to see if the result is valid

    double p1_from_p0 = p0 + v0*t1;
    double p2_from_p0 = p1_from_p0 + v1*(t2-t1) + 0.5*a1*pow(t2-t1,2);

    bool valid = true;
    uint8_t reason = 0;

    if (v0 == 0) {
        valid = false;
        reason += 1;
    }

    if (t1 < -0.00001f) {
        valid = false;
        reason += 2;
    }

    if (t2 - t1 < -0.00001f) {
        valid = false;
        reason += 4;
    }

    if (fabs(p2_from_p0 - p2) > p_threshold) {
        valid = false;
        reason += 8;
    }

    if (isnan(t1) || isnan(t2)) {
        valid = false;
        reason = 16;
    }

    if (v0 > limit_pos.vel || v0 < limit_neg.vel) {
        valid = false;
        reason += 32;
    }

    // if result is valid, output the path

    if (valid) {
        t_out[1] = 0;
        a_out[1] = 0;
        t_out[2] = t1;
        a_out[2] = an;
        t_out[3] = t2;
        a_out[3] = 0;

        return true;
    }

    else {
        return false;
    }
}

bool Motion1DPositionVelocityAccelDouble::target_has_changed_since_last_run(motion_state_s* target_state) {
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

void Motion1DPositionVelocityAccelDouble::determine_target_mode() {

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

    // if we're in velocity mode and our velocity is beyond the limit, get velocity back in-range
    if (isnan(user.pos) && (cur.vel > limit_pos.vel || cur.vel < limit_neg.vel)) {
        target_mode = velocity;
        target.vel = minmax(cur.vel, limit_neg.vel, limit_pos.vel);
        using_velocity_limit = true;
    }
    else {
        using_velocity_limit = false;
    }

    // if we're in velocity mode but going to overrun a position limit, switch to position mode at the limit

    motion_state_s next = compute_next_state_from_accel(&cur, cur.acc, dt);
    double stopping_acc = next.vel > 0 ? limit_neg.acc : limit_pos.acc;
    double time_to_make_vel_zero = abs(next.vel / stopping_acc);
    double position_if_stopped_now = next.pos + next.vel * time_to_make_vel_zero + 0.5 * stopping_acc * pow(time_to_make_vel_zero, 2);

    bool position_in_bounds = cur.pos > limit_neg.pos && cur.pos < limit_pos.pos;

    if (!using_position_limit && position_in_bounds && target_mode == velocity) {
        if (position_if_stopped_now > limit_pos.pos) {
            using_position_limit = true;
            position_limit = limit_pos.pos;
            arrived_at_position_target = false;
        }
        if (position_if_stopped_now < limit_neg.pos) {
            using_position_limit = true;
            position_limit = limit_neg.pos;
            arrived_at_position_target = false;
        }
    }

    if (using_position_limit) {
        target_mode = position;
        target.pos = position_limit;
        target.vel = 0;
    }

}

bool Motion1DPositionVelocityAccelDouble::check_input_and_config_errors() {

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

bool Motion1DPositionVelocityAccelDouble::check_output_errors() {

    bool error_found = false;

    error_code.bits.cur_pos_nan = isnan(cur.pos);
    if (error_code.bits.cur_pos_nan) error_found = true;

    error_code.bits.cur_vel_nan = isnan(cur.vel);
    if (error_code.bits.cur_vel_nan) error_found = true;

    error_code.bits.cur_acc_nan = isnan(cur.acc);
    if (error_code.bits.cur_acc_nan) error_found = true;

    error_code.bits.cur_jrk_nan = isnan(cur.jrk);
    if (error_code.bits.cur_jrk_nan) error_found = true;

	error_code.bits.no_solver_found = best_solver == 0;
	if (error_code.bits.no_solver_found) error_found = true;

    return !error_found;
}

