#include "Motion1DPositionVelocityAccelJerkDouble.h"
#include <cstring>
#include <complex>

using namespace std;

// note: multiple targets can be set at once.
// when this happens, targetting is controlled by the highest-order parameter.
// lower-order parameters are interpreted as the target values for those orders at the instant the higher-order target finishes.
// so pos=5, vel=10, acc=2 means:
//  - seek pos 4
//  - arrive at pos 5 with vel 10 and acc of 2
//  - continue with a constant acc of 2 forever (leaving pos 5)
//  - this may hit a pos or vel limit, where it will switch to pos or vel mode to not overrun the limit

void Motion1DPositionVelocityAccelJerkDouble::configure_position_limits(double _limit_pos_neg, double _limit_pos_pos) {
    configure_position_limits_int(_limit_pos_neg, _limit_pos_pos);
}

void Motion1DPositionVelocityAccelJerkDouble::configure_velocity_limits(double _limit_vel_neg, double _limit_vel_pos) {
    configure_velocity_limits_int(_limit_vel_neg, _limit_vel_pos);
    // note: we are deliberately misaligning the motion orders here.
    // see get_jerk_for_accel_mode() and get_jerk_for_velocity_mode() for how these are used.
    motion_pva.configure_position_limits(_limit_vel_neg, _limit_vel_pos);

}

void Motion1DPositionVelocityAccelJerkDouble::configure_acceleration_limits(double _limit_acc_neg, double _limit_acc_pos) {
    configure_acceleration_limits_int(_limit_acc_neg, _limit_acc_pos);
    // note: we are deliberately misaligning the motion orders here.
    // see get_jerk_for_accel_mode() and get_jerk_for_velocity_mode() for how these are used.
    motion_pv.configure_position_limits(_limit_acc_neg, _limit_acc_pos);
    motion_pva.configure_velocity_limits(_limit_acc_neg, _limit_acc_pos);

}

void Motion1DPositionVelocityAccelJerkDouble::configure_jerk_limits(double _limit_jrk) {
    configure_jerk_limits_int(_limit_jrk);
    // note: we are deliberately misaligning the motion orders here.
    // see get_jerk_for_accel_mode() and get_jerk_for_velocity_mode() for how these are used.
    motion_pv.configure_velocity_limits(-_limit_jrk, _limit_jrk);
    motion_pva.configure_acceleration_limits(-_limit_jrk, _limit_jrk);
}

void Motion1DPositionVelocityAccelJerkDouble::configure_dt(double _dt) {
    configure_dt_int(_dt);
    motion_pv.configure_dt(_dt);
    motion_pva.configure_dt(_dt);
}

void Motion1DPositionVelocityAccelJerkDouble::reset_state() {
	// since there's no constructor / init function, the initial values of these variables must match the below values
	using_velocity_limit = false;
	velocity_limit = 0;
	using_position_limit = false;
	position_limit = 0;
	arrived_at_velocity_target = false;
	arrived_at_velocity_target_now = false;
	arrived_at_position_target = false;
	arrived_at_position_target_now = false;

    motion_pv.reset_state();
    motion_pva.reset_state();
}

void Motion1DPositionVelocityAccelJerkDouble::run_next_timestep() {

	// if there are any configuration or user input errors, report them and exit immediately.
	// these must be corrected at design-time by fixing the calling code.

	if (!check_input_and_config_errors()) return;

	if (target_has_changed_since_last_run(&user)) {
		reset_state();
	}

    // store last state temporary so that they can be used to calculate the new values

	last = cur;

    // figure out which dimension of motion we're targeting - position, velocity or acceleration

	determine_target_mode();

	// run the relevant controller to come up with the new jerk value, which will determine the next state of motion

	double jerk = NAN;
	double computed_time_remaining = 0;
	double temp = 0;

    switch (target_mode) {

	case target_mode_e::acceleration:
		jerk = get_jerk_for_accel_mode(&last, &target, &computed_time_remaining);
		cur = compute_next_state_from_jerk(&last, jerk, dt);
        break;

	case target_mode_e::velocity:
		jerk = get_jerk_for_velocity_mode(&last, &target, &computed_time_remaining, &temp);
		cur = compute_next_state_from_jerk(&last, jerk, dt);
		if (computed_time_remaining < dt && !arrived_at_velocity_target) {
			arrived_at_velocity_target = true;
			arrived_at_velocity_target_now = true;
		}
		break;

	case target_mode_e::position:
		cur = get_state_for_position_mode(&last, &target, &computed_time_remaining);
		if (computed_time_remaining < dt && !arrived_at_position_target) {
			arrived_at_position_target = true;
			arrived_at_position_target_now = true;
			arrived_at_velocity_target = true;
			arrived_at_velocity_target_now = true;
		}
		break;

	default:
		break;

	}

    cur_time_remaining = (using_velocity_limit || using_position_limit) ? NAN : ((arrived_at_velocity_target || arrived_at_position_target) ? 0 : computed_time_remaining);

    // if we just hit the a target this frame, adjust our state so that it exactly match the target.
    // this is designed to correct for doubleing-point math errors only - anything bigger than that means that the equations are wrong and should be fixed!

    if (arrived_at_velocity_target_now) {
    	arrived_at_velocity_target_now = false;
    	cur.acc = isnan(target.acc) ? 0 : target.acc;
    	cur.vel = target.vel;
    }

    if (arrived_at_position_target_now) {
    	arrived_at_position_target_now = false;
    	cur.acc = isnan(target.acc) ? 0 : target.acc;
    	cur.vel = isnan(target.vel) ? 0 : target.vel;
    	cur.pos = target.pos;
    }

    // check for errors in the output signals

    check_output_errors();
}

double Motion1DPositionVelocityAccelJerkDouble::get_jerk_for_accel_mode(motion_state_s* current_state, motion_state_s* target_state, double* time_remaining) {

    // we use the 2nd-order module for this.
    // since it works in position/velocity and we have jerk/accel, everything has to be shifted by two orders

    motion_pv.jump_to(current_state->acc);
    motion_pv.set_target(isnan(target_state->acc) ? 0 : target_state->acc);
    motion_pv.reset_state();
    motion_pv.run_next_timestep();
    double jerk = motion_pv.get_velocity();
    *time_remaining = motion_pv.get_time_remaining();

    best_solver = -10;

	return jerk;
}

double Motion1DPositionVelocityAccelJerkDouble::get_jerk_for_velocity_mode(motion_state_s* current_state, motion_state_s* target_state, double* time_remaining, double* final_position) {

    // we use the 3nd-order module for this.
    // since it works in position/velocity/accel and we have jerk/accel/velocity, everything has to be shifted by one order

    motion_pva.jump_to(isnan(current_state->vel) ? 0 : current_state->vel, current_state->acc);
    motion_pva.set_target(isnan(target_state->vel) ? 0 : target_state->vel, target_state->acc);
    motion_pva.reset_state();
    motion_pva.run_next_timestep();
    double jerk = motion_pva.get_acceleration();
    *time_remaining = motion_pva.get_time_remaining();

    motion_state_s final_state = motion_pva.get_state_at_time(*time_remaining);
    *final_position = final_state.pos;

    best_solver = -9;

    return jerk;
}

Motion1DInterfaceDouble::motion_state_s Motion1DPositionVelocityAccelJerkDouble::get_state_for_position_mode(motion_state_s* current_state, motion_state_s* target_state, double* time_remaining) {

	double jp1=0, jn1=0, ap1=0, an1=0, vmax1=0;
	double jp2=0, jn2=0, ap2=0, an2=0, vmax2=0;
	double t[8] = {0}, j[8] = {0};
	double best_t[8] = {0}, best_j[8] = {0};
	best_solver = 0;
	best_t[7] = INFINITY;

	// try all of the different possible curves that could solve the motion problem, in order by least segments to most

	get_jp_jn_ap_an_vmax_option(1, &jp1, &jn1, &ap1, &an1, &vmax1);
	get_jp_jn_ap_an_vmax_option(2, &jp2, &jn2, &ap2, &an2, &vmax2);

	if (solve_position_last_1_phase(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 1);
	if (solve_position_last_1_phase(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -1);
//	if (solve_position_last_2_phase(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 2);
//	if (solve_position_last_2_phase(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -2);
	if (solve_position_last_3_phase(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 3);
	if (solve_position_last_3_phase(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -3);
//	if (solve_position_last_4_phase(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 4);
//	if (solve_position_last_4_phase(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -4);
	if (solve_position_last_5_phase(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 5);
	if (solve_position_last_5_phase(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -5);
//	if (solve_position_last_6_phase(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 6);
//	if (solve_position_last_6_phase(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -6);
	if (solve_position_last_7_phase(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 7);
	if (solve_position_last_7_phase(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -7);
	if (solve_position_last_3_phase_no_6(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 36);
	if (solve_position_last_3_phase_no_6(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -36);
//	if (solve_position_last_4_phase_no_6(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 46);
//	if (solve_position_last_4_phase_no_6(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -46);
	if (solve_position_last_5_phase_no_6(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 56);
	if (solve_position_last_5_phase_no_6(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -56);
//	if (solve_position_last_6_phase_no_6(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 66);
//	if (solve_position_last_6_phase_no_6(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -66);
	if (solve_position_last_7_phase_no_6(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 76);
	if (solve_position_last_7_phase_no_6(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -76);
	if (solve_position_last_7_phase_no_2(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 72);
	if (solve_position_last_7_phase_no_2(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -72);
	if (solve_position_last_7_phase_no_2_no_6(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 726);
	if (solve_position_last_7_phase_no_2_no_6(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -726);
	if (solve_position_last_7_phase_no_4(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 74);
	if (solve_position_last_7_phase_no_4(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -74);
    if (solve_position_last_7_phase_no_4_no_2(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 742);
    if (solve_position_last_7_phase_no_4_no_2(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -742);
    if (solve_position_last_7_phase_no_4_no_6(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 746);
    if (solve_position_last_7_phase_no_4_no_6(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -746);
    if (solve_position_last_5_phase_no_4_no_6(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 546);
    if (solve_position_last_5_phase_no_4_no_6(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -546);
	if (solve_position_last_7_phase_no_4_no_2_no_6(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 7426);
    if (solve_position_last_7_phase_no_4_no_2_no_6(current_state, target_state, jp2, jn2, ap2, an2, vmax2, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, -7426);
    //if (solve_position_test(current_state, target_state, jp1, jn1, ap1, an1, vmax1, t, j)) update_best_solver(t, j, best_t, best_j, &best_solver, 0.1);

	debug[5] = best_solver;
    motion_state_s next_state;

	if (best_solver) {
        // store the solution for use by get_state_at_time()

        memcpy(last_solution_t, best_t, sizeof(best_t));
        memcpy(last_solution_j, best_j, sizeof(best_j));
        memcpy(&last_ics, current_state, sizeof(last_ics));

        // next state determination

        next_state = get_state_at_time(dt);

        *time_remaining = best_t[7];
    }

	else {
	    double temp;
	    double jerk_for_vel = get_jerk_for_velocity_mode(current_state, target_state, &temp, &temp);
	    *time_remaining = 999;
	    debug[1] = jerk_for_vel;
	    next_state = compute_next_state_from_jerk(current_state, jerk_for_vel, dt);
	}

	return next_state;

}

void Motion1DPositionVelocityAccelJerkDouble::update_best_solver(double t[], double j[], double best_t[], double best_j[], int* best_solver, int this_solver) {

	// this is a helper method for get_state_for_position_mode().
	// it determines if the current solution is better than the previous best solution, and updates data accordingly.
	// if the new finishing time is quicker it becomes the best.
	// if the new finishing time ties with the best (with some margin for doubleing-point error), then the solution with fewer stages wins.

	bool new_is_best = false;

	if (abs(t[7] - best_t[7]) < 1e-6) {
		int phases_old = 0;
		int phases_new = 0;
		for (int i=0; i<8; i++) {
			if (t[i] != 0) phases_new++;
			if (best_t[i] != 0) phases_old++;
		}
		if (phases_new < phases_old) new_is_best = true;
	}

	else if (t[7] < best_t[7]) {
		new_is_best = true;
	}

	if (new_is_best) {
		memcpy(best_t, t, 8 * sizeof(double));
		memcpy(best_j, j, 8 * sizeof(double));
		*best_solver = this_solver;
	}
}

void Motion1DPositionVelocityAccelJerkDouble::get_jp_jn_ap_an_vmax_option(int option, double* jp, double* jn, double* ap, double* an, double* vmax) {

	switch (option) {

	case 1:
		*jp = limit_pos.jrk;
		*jn = limit_neg.jrk;
		*ap = limit_pos.acc;
		*an = limit_neg.acc;
		*vmax = limit_pos.vel;
		break;

	case 2:
		*jp = limit_neg.jrk;
		*jn = limit_pos.jrk;
		*ap = limit_neg.acc;
		*an = limit_pos.acc;
		*vmax = limit_neg.vel;
		break;

	}
}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_1_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t6 = 0;
	double j6 = jp;
	double a6 = current_state->acc;
	double v6 = current_state->vel;
	double p6 = current_state->pos;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double t67 = dt_from_da_j(a6, a7, j6);

	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v7_from_t7 = next_v(v6, a6, j6, t67);
	double p7_from_t7 = next_p(p6, v6, a6, j6, t67);
	double v7_error = abs(v7_from_t7 - v7);
	double p7_error = abs(p7_from_t7 - p7);

	bool dts_positive = t67 > -1e-6;

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v7_error < v_threshold && p7_error < p_threshold && dts_positive) {

		t_out[7] = t7;

		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_2_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t5 = 0;
	double j5 = 0;
	double a5 = current_state->acc;
	double v5 = current_state->vel;
	double p5 = current_state->pos;
	double j6 = jp;
	double a6 = an;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double t67 = dt_from_da_j(a6, a7, j6);
	double v6 = last_v(v7, a7, j6, t67);
	double t56 = dt_from_dv_a(v5, v6, a5);

	double t6 = t5 + t56;
	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v6_from_t5 = next_v(v5, a5, j5, t56);
	double p6_from_t5 = next_p(p5, v5, a5, j5, t56);
	double p6_from_t7 = last_p(p7, v7, a7, j6, t67);
	double v6_error = abs(v6_from_t5 - v6);
	double p6_error = abs(p6_from_t7 - p6_from_t5);

	bool dts_positive = t56 > -1e-6 && t67 > -1e-6;

    bool a5_is_max = fabs(a5 - an) < 1e-6;

    // if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v6_error < v_threshold && p6_error < p_threshold && dts_positive && a5_is_max) {

		t_out[6] = t6;
		t_out[7] = t7;

		j_out[5] = j5;
		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_3_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t4 = 0;
	double j4 = jn;
	double a4 = current_state->acc;
	double v4 = current_state->vel;
	double p4 = current_state->pos;
	double j5 = 0;
	double a5 = an;
	double j6 = jp;
	double a6 = an;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double t67 = dt_from_da_j(a6, a7, j6);
	double v6 = last_v(v7, a7, j6, t67);
	double p6 = last_p(p7, v7, a7, j6, t67);
	double t45 = dt_from_da_j(a4, a5, j4);
	double v5 = next_v(v4, a4, j4, t45);
	double p5 = next_p(p4, v4, a4, j4, t45);
	double t56 = dt_from_dv_a(v5, v6, a5);

	double t5 = t4 + t45;
	double t6 = t5 + t56;
	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v6_from_t5 = next_v(v5, a5, j5, t56);
	double p6_from_t5 = next_p(p5, v5, a5, j5, t56);
	double v6_error = abs(v6_from_t5 - v6);
	double p6_error = abs(p6_from_t5 - p6);

	bool dts_positive = t45 > -1e-6 && t56 > -1e-6 && t67 > -1e-6;

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v6_error < v_threshold && p6_error < p_threshold && dts_positive) {

		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[4] = j4;
		j_out[5] = j5;
		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_4_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t3 = 0;
	double j3 = 0;
	double a3 = current_state->acc;
	double v3 = current_state->vel;
	double p3 = current_state->pos;
	double j4 = jn;
	double a4 = 0;
	double v4 = vmax;
	double j5 = 0;
	double a5 = an;
	double j6 = jp;
	double a6 = an;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double t67 = dt_from_da_j(a6, a7, j6);
	double v6 = last_v(v7, a7, j6, t67);
	double p6 = last_p(p7, v7, a7, j6, t67);
	double t45 = dt_from_da_j(a4, a5, j4);
	double v5 = next_v(v4, a4, j4, t45);
	double t56 = dt_from_dv_a(v5, v6, a5);
	double p5 = last_p(p6, v6, a6, j5, t56);
	double p4 = last_p(p5, v5, a5, j4, t45);
	double t34 = dt_from_dp_v(p3, p4, v3);

	double t4 = t3 + t34;
	double t5 = t4 + t45;
	double t6 = t5 + t56;
	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v4_from_t3 = next_v(v3, a3, j3, t34);
	double p4_from_t3 = next_p(p3, v3, a3, j3, t34);
	double v4_error = abs(v4_from_t3 - v4);
	double p4_error = abs(p4_from_t3 - p4);

	bool dts_positive = t34 > -1e-6 && t45 > -1e-6 && t56 > -1e-6 && t67 > -1e-6;

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v4_error < v_threshold && p4_error < p_threshold && dts_positive) {

		t_out[4] = t4;
		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[3] = j3;
		j_out[4] = j4;
		j_out[5] = j5;
		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_5_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t2 = 0;
	double j2 = jn;
	double a2 = current_state->acc;
	double v2 = current_state->vel;
	double p2 = current_state->pos;
	double j3 = 0;
	double a3 = 0;
	double v3 = vmax;
	double j4 = jn;
	double a4 = 0;
	double v4 = vmax;
	double j5 = 0;
	double a5 = an;
	double j6 = jp;
	double a6 = an;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double t67 = dt_from_da_j(a6, a7, j6);
	double v6 = last_v(v7, a7, j6, t67);
	double p6 = last_p(p7, v7, a7, j6, t67);
	double t45 = dt_from_da_j(a4, a5, j4);
	double v5 = next_v(v4, a4, j4, t45);
	double t56 = dt_from_dv_a(v5, v6, a5);
	double p5 = last_p(p6, v6, a6, j5, t56);
	double p4 = last_p(p5, v5, a5, j4, t45);
	double t23 = dt_from_da_j(a2, a3, j2);
	double p3 = next_p(p2, v2, a2, j2, t23);
	double t34 = dt_from_dp_v(p3, p4, v3);

	double t3 = t2 + t23;
	double t4 = t3 + t34;
	double t5 = t4 + t45;
	double t6 = t5 + t56;
	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v3_from_t2 = next_v(v2, a2, j2, t23);
	double p3_from_t4 = last_p(p4, v4, a4, j3, t34);
	double v3_error = abs(v3_from_t2 - v3);
	double p3_error = abs(p3_from_t4 - p3);

	bool dts_positive = t23 > -1e-6 && t34 > -1e-6 && t45 > -1e-6 && t56 > -1e-6 && t67 > -1e-6;

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v3_error < v_threshold && p3_error < p_threshold && dts_positive) {

		t_out[3] = t3;
		t_out[4] = t4;
		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[2] = j2;
		j_out[3] = j3;
		j_out[4] = j4;
		j_out[5] = j5;
		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_6_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t1 = 0;
	double j1 = 0;
	double a1 = current_state->acc;
	double v1 = current_state->vel;
	double p1 = current_state->pos;
	double j2 = jn;
	double a2 = ap;
	double j3 = 0;
	double a3 = 0;
	double v3 = vmax;
	double j4 = jn;
	double a4 = 0;
	double v4 = vmax;
	double j5 = 0;
	double a5 = an;
	double j6 = jp;
	double a6 = an;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double t67 = dt_from_da_j(a6, a7, j6);
	double v6 = last_v(v7, a7, j6, t67);
	double p6 = last_p(p7, v7, a7, j6, t67);
	double t45 = dt_from_da_j(a4, a5, j4);
	double v5 = next_v(v4, a4, j4, t45);
	double t56 = dt_from_dv_a(v5, v6, a5);
	double p5 = last_p(p6, v6, a6, j5, t56);
	double p4 = last_p(p5, v5, a5, j4, t45);
	double t23 = dt_from_da_j(a2, a3, j2);
	double v2 = last_v(v3, a3, j2, t23);
	double t12 = dt_from_dv_a(v1, v2, a1);
	double p2 = next_p(p1, v1, a1, j1, t12);
	double p3 = next_p(p2, v2, a2, j2, t23);
	double t34 = dt_from_dp_v(p3, p4, v3);

	double t2 = t1 + t12;
	double t3 = t2 + t23;
	double t4 = t3 + t34;
	double t5 = t4 + t45;
	double t6 = t5 + t56;
	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v2_from_t1 = next_v(v1, a1, j1, t12);
	double p3_from_t4 = last_p(p4, v4, a4, j3, t34);
	double v2_error = abs(v2_from_t1 - v2);
	double p3_error = abs(p3_from_t4 - p3);
	double a1_error = abs(a1 - ap);

	bool dts_positive = t12 > -1e-6 && t23 > -1e-6 && t34 > -1e-6 && t45 > -1e-6 && t56 > -1e-6 && t67 > -1e-6;

    bool a1_is_max = fabs(a1 - ap) < 1e-6;

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v2_error < v_threshold && p3_error < p_threshold && a1_error < a_threshold && dts_positive && a1_is_max) {

		t_out[2] = t2;
		t_out[3] = t3;
		t_out[4] = t4;
		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[1] = j1;
		j_out[2] = j2;
		j_out[3] = j3;
		j_out[4] = j4;
		j_out[5] = j5;
		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_7_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t0 = 0;
	double j0 = jp;
	double a0 = current_state->acc;
	double v0 = current_state->vel;
	double p0 = current_state->pos;
	double j1 = 0;
	double a1 = ap;
	double j2 = jn;
	double a2 = ap;
	double j3 = 0;
	double a3 = 0;
	double v3 = vmax;
	double j4 = jn;
	double a4 = 0;
	double v4 = vmax;
	double j5 = 0;
	double a5 = an;
	double j6 = jp;
	double a6 = an;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double t67 = dt_from_da_j(a6, a7, j6);
	double v6 = last_v(v7, a7, j6, t67);
	double p6 = last_p(p7, v7, a7, j6, t67);
	double t45 = dt_from_da_j(a4, a5, j4);
	double v5 = next_v(v4, a4, j4, t45);
	double t56 = dt_from_dv_a(v5, v6, a5);
	double p5 = last_p(p6, v6, a6, j5, t56);
	double p4 = last_p(p5, v5, a5, j4, t45);
	double t23 = dt_from_da_j(a2, a3, j2);
	double v2 = last_v(v3, a3, j2, t23);
	double t01 = dt_from_da_j(a0, a1, j0);
	double p1 = next_p(p0, v0, a0, j0, t01);
	double v1 = next_v(v0, a0, j0, t01);
	double t12 = dt_from_dv_a(v1, v2, a1);
	double p2 = next_p(p1, v1, a1, j1, t12);
	double p3 = next_p(p2, v2, a2, j2, t23);
	double t34 = dt_from_dp_v(p3, p4, v3);

	double t1 = t0 + t01;
	double t2 = t1 + t12;
	double t3 = t2 + t23;
	double t4 = t3 + t34;
	double t5 = t4 + t45;
	double t6 = t5 + t56;
	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v2_from_t1 = next_v(v1, a1, j1, t12);
	double p3_from_t4 = last_p(p4, v4, a4, j3, t34);
	double v2_error = abs(v2_from_t1 - v2);
	double p3_error = abs(p3_from_t4 - p3);

	bool dts_positive = t01 > -1e-6 && t12 > -1e-6 && t23 > -1e-6 && t34 > -1e-6 && t45 > -1e-6 && t56 > -1e-6 && t67 > -1e-6;

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v2_error < v_threshold && p3_error < p_threshold && dts_positive) {

		t_out[1] = t1;
		t_out[2] = t2;
		t_out[3] = t3;
		t_out[4] = t4;
		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[0] = j0;
		j_out[1] = j1;
		j_out[2] = j2;
		j_out[3] = j3;
		j_out[4] = j4;
		j_out[5] = j5;
		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_3_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t4 = 0;
	double j4 = jn;
	double a4 = current_state->acc;
	double v4 = current_state->vel;
	double p4 = current_state->pos;
	double j56 = jp;
	double j6 = jp;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double x1 = sqrt((j4-j56)*(-j56*a4*a4 + j4*a7*a7 + 2*j4*j56*v4 - 2*j4*j56*v7));
	double t67a = ( x1 + a7*j4 - a7*j56) / (j4*j56 - j56*j56);
	double t67b = (-x1 + a7*j4 - a7*j56) / (j4*j56 - j56*j56);
	double t67 = max(t67a, t67b);
	double t456 = (a7 - a4 - j56*t67) / j4;
	double a56 = next_a(a4, j4, t456);

	double t5 = t4 + t456;
	double t6 = t5;
	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v56_from_t4 = next_v(v4, a4, j4, t456);
	double p56_from_t4 = next_p(p4, v4, a4, j4, t456);
	double v56_from_t7 = last_v(v7, a7, j6, t67);
	double p56_from_t7 = last_p(p7, v7, a7, j6, t67);
	double v56_error = abs(v56_from_t4 - v56_from_t7);
	double p56_error = abs(p56_from_t4 - p56_from_t7);

	bool dts_positive = t456 > -1e-6 && t67 > -1e-6;
	bool dts_real = !isnan(t456) && !isinf(t456) && !isnan(t67) && !isinf(t67);
	bool a56_within_limit = abs(a56) <= abs(an);

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v56_error < v_threshold && p56_error < p_threshold && dts_positive && dts_real && a56_within_limit) {

		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[4] = j4;
		j_out[5] = j56;
		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_4_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t3 = 0;
	double j3 = 0;
	double a3 = 0;
	double v3 = current_state->vel;
	double p3 = current_state->pos;
	double j4 = jn;
	double a4 = 0;
	double v4 = current_state->vel;
	double j56 = jp;
	double j6 = jp;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double x1 = sqrt((j4-j56)*(-j56*a4*a4 + j4*a7*a7 + 2*j4*j56*v4 - 2*j4*j56*v7));
	double t67a = ( x1 + a7*j4 - a7*j56) / (j4*j56 - j56*j56);
	double t67b = (-x1 + a7*j4 - a7*j56) / (j4*j56 - j56*j56);
	double t67 = max(t67a, t67b);
	double t456 = (a7 - a4 - j56*t67) / j4;
	double p56 = last_p(p7, v7, a7, j6, t67);
	double v56 = last_v(v7, a7, j6, t67);
	double a56 = last_a(a7, j6, t67);
	double p4 = last_p(p56, v56, a56, j4, t456);
	double t34 = dt_from_dp_v(p3, p4, v3);

	double t4 = t3 + t34;
	double t5 = t4 + t456;
	double t6 = t5;
	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v56_from_t4 = next_v(v4, a4, j4, t456);
	double v56_error = abs(v56_from_t4 - v56);
	double a3_error = abs(current_state->acc - a3);
	double v3_error = abs(v3 - vmax);

	bool dts_positive = t34 > -1e-6 && t456 > -1e-6 && t67 > -1e-6;
	bool dts_real = !isnan(t34) && !isinf(t34) && !isnan(t456) && !isinf(t456) && !isnan(t67) && !isinf(t67);
	bool a56_within_limit = abs(a56) <= abs(an);

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v56_error < v_threshold && a3_error < a_threshold && v3_error < v_threshold && dts_positive && dts_real && a56_within_limit) {

		t_out[4] = t4;
		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[3] = j3;
		j_out[4] = j4;
		j_out[5] = j56;
		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}
}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_5_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t2 = 0;
	double j2 = jn;
	double a2 = current_state->acc;
	double v2 = current_state->vel;
	double p2 = current_state->pos;
	double j3 = 0;
	double a3 = 0;
	double j4 = jn;
	double a4 = 0;
	double j56 = jp;
	double j6 = jp;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double t23 = dt_from_da_j(a2, a3, j2);
	double v3 = next_v(v2, a2, j2, t23);
	double v4 = v3;
	double p3 = next_p(p2, v2, a2, j2, t23);

	double x1 = sqrt((j4-j56)*(-j56*a4*a4 + j4*a7*a7 + 2*j4*j56*v4 - 2*j4*j56*v7));
	double t67a = ( x1 + a7*j4 - a7*j56) / (j4*j56 - j56*j56);
	double t67b = (-x1 + a7*j4 - a7*j56) / (j4*j56 - j56*j56);
	double t67 = max(t67a, t67b);
	double t456 = (a7 - a4 - j56*t67) / j4;
	double p56 = last_p(p7, v7, a7, j6, t67);
	double v56 = last_v(v7, a7, j6, t67);
	double a56 = last_a(a7, j6, t67);
	double p4 = last_p(p56, v56, a56, j4, t456);
	double t34 = dt_from_dp_v(p3, p4, v3);

	double t3 = t2 + t23;
	double t4 = t3 + t34;
	double t5 = t4 + t456;
	double t6 = t5;
	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v56_from_t4 = next_v(v4, a4, j4, t456);
	double v56_error = abs(v56_from_t4 - v56);
	double v3_error = abs(v3 - vmax);

	bool dts_positive = t23 > -1e-6 && t34 > -1e-6 && t456 > -1e-6 && t67 > -1e-6;
	bool dts_real = !isnan(t23) && !isinf(t23) && !isnan(t34) && !isinf(t34) && !isnan(t456) && !isinf(t456) && !isnan(t67) && !isinf(t67);
	bool a56_within_limit = abs(a56) <= abs(an);

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v56_error < v_threshold && v3_error < v_threshold && dts_positive && dts_real && a56_within_limit) {

		t_out[3] = t3;
		t_out[4] = t4;
		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[2] = j2;
		j_out[3] = j3;
		j_out[4] = j4;
		j_out[5] = j56;
		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_6_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t1 = 0;
	double j1 = 0;
	double a1 = current_state->acc;
	double v1 = current_state->vel;
	double p1 = current_state->pos;
	double a2 = ap;
	double j2 = jn;
	double j3 = 0;
	double a3 = 0;
	double v3 = vmax;
	double j4 = jn;
	double a4 = 0;
	double v4 = vmax;
	double j56 = jp;
	double j6 = jp;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double t23 = dt_from_da_j(a2, a3, j2);
	double v2 = last_v(v3, a3, j2, t23);
	double t12 = dt_from_dv_a(v1, v2, a1);
	double p2 = next_p(p1, v1, a1, j1, t12);
	double p3 = next_p(p2, v2, a2, j2, t23);
	double x1 = sqrt((j4-j56)*(-j56*a4*a4 + j4*a7*a7 + 2*j4*j56*v4 - 2*j4*j56*v7));
	double t67a = ( x1 + a7*j4 - a7*j56) / (j4*j56 - j56*j56);
	double t67b = (-x1 + a7*j4 - a7*j56) / (j4*j56 - j56*j56);
	double t67 = max(t67a, t67b);
	double t456 = (a7 - a4 - j56*t67) / j4;
	double p56 = last_p(p7, v7, a7, j6, t67);
	double v56 = last_v(v7, a7, j6, t67);
	double a56 = last_a(a7, j6, t67);
	double p4 = last_p(p56, v56, a56, j4, t456);
	double t34 = dt_from_dp_v(p3, p4, v3);

	double t2 = t1 + t12;
	double t3 = t2 + t23;
	double t4 = t3 + t34;
	double t5 = t4 + t456;
	double t6 = t5;
	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v56_from_t4 = next_v(v4, a4, j4, t456);
	double v56_error = abs(v56_from_t4 - v56);
	double a1_error = abs(a1 - ap);

	bool dts_positive = t12 > -1e-6 && t23 > -1e-6 && t34 > -1e-6 && t456 > -1e-6 && t67 > -1e-6;
	bool dts_real = !isnan(t12) && !isinf(t12) && !isnan(t23) && !isinf(t23) && !isnan(t34) && !isinf(t34) && !isnan(t456) && !isinf(t456) && !isnan(t67) && !isinf(t67);
	bool a56_within_limit = abs(a56) <= abs(an);

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v56_error < v_threshold && a1_error < a_threshold && dts_positive && dts_positive && a56_within_limit) {

		t_out[2] = t2;
		t_out[3] = t3;
		t_out[4] = t4;
		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[1] = j1;
		j_out[2] = j2;
		j_out[3] = j3;
		j_out[4] = j4;
		j_out[5] = j56;
		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_7_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// todo: not tested yet

	// initial conditions

	double t0 = 0;
	double j0 = jp;
	double a0 = current_state->acc;
	double v0 = current_state->vel;
	double p0 = current_state->pos;
	double j1 = 0;
	double a1 = ap;
	double a2 = ap;
	double j2 = jn;
	double j3 = 0;
	double a3 = 0;
	double v3 = vmax;
	double j4 = jn;
	double a4 = 0;
	double v4 = vmax;
	double j56 = jp;
	double j6 = jp;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double t01 = dt_from_da_j(a0, a1, j0);
	double v1 = next_v(v0, a0, j0, t01);
	double p1 = next_p(p0, v0, a0, j0, t01);
	double t23 = dt_from_da_j(a2, a3, j2);
	double v2 = last_v(v3, a3, j2, t23);
	double t12 = dt_from_dv_a(v1, v2, a1);
	double p2 = next_p(p1, v1, a1, j1, t12);
	double p3 = next_p(p2, v2, a2, j2, t23);
	double x1 = sqrt((j4-j56)*(-j56*a4*a4 + j4*a7*a7 + 2*j4*j56*v4 - 2*j4*j56*v7));
	double t67a = ( x1 + a7*j4 - a7*j56) / (j4*j56 - j56*j56);
	double t67b = (-x1 + a7*j4 - a7*j56) / (j4*j56 - j56*j56);
	double t67 = max(t67a, t67b);
	double t456 = (a7 - a4 - j56*t67) / j4;
	double p56 = last_p(p7, v7, a7, j6, t67);
	double v56 = last_v(v7, a7, j6, t67);
	double a56 = last_a(a7, j6, t67);
	double p4 = last_p(p56, v56, a56, j4, t456);
	double t34 = dt_from_dp_v(p3, p4, v3);

	double t1 = t0 + t01;
	double t2 = t1 + t12;
	double t3 = t2 + t23;
	double t4 = t3 + t34;
	double t5 = t4 + t456;
	double t6 = t5;
	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v56_from_t4 = next_v(v4, a4, j4, t456);
	double v56_error = abs(v56_from_t4 - v56);
	double a1_error = abs(a1 - ap);
	bool a56_within_limit = abs(a56) <= abs(an);

	bool dts_positive = t01 > -1e-6 && t12 > -1e-6 && t23 > -1e-6 && t34 > -1e-6 && t456 > -1e-6 && t67 > -1e-6;

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v56_error < v_threshold && a1_error < a_threshold && dts_positive && a56_within_limit) {

		t_out[1] = t1;
		t_out[2] = t2;
		t_out[3] = t3;
		t_out[4] = t4;
		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[0] = j0;
		j_out[1] = j1;
		j_out[2] = j2;
		j_out[3] = j3;
		j_out[4] = j4;
		j_out[5] = j56;
		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_7_phase_no_2(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t0 = 0;
	double j0 = jp;
	double a0 = current_state->acc;
	double v0 = current_state->vel;
	double p0 = current_state->pos;
	double j12 = jn;
	double j3 = 0;
	double a3 = 0;
	double v3 = vmax;
	double j4 = jn;
	double a4 = 0;
	double v4 = vmax;
	double j5 = 0;
	double a5 = an;
	double j6 = jp;
	double a6 = an;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double t67 = dt_from_da_j(a6, a7, j6);
	double v6 = last_v(v7, a7, j6, t67);
	double p6 = last_p(p7, v7, a7, j6, t67);
	double t45 = dt_from_da_j(a4, a5, j4);
	double v5 = next_v(v4, a4, j4, t45);
	double t56 = dt_from_dv_a(v5, v6, a5);
	double p5 = last_p(p6, v6, a6, j5, t56);
	double p4 = last_p(p5, v5, a5, j4, t45);

	double x1 = sqrt((j0-j12) * (-j12*a0*a0 + j0*a3*a3 + 2*j0*j12*v0 - 2*j0*j12*v3));
	double t23a = ( x1 + a3*j0 - a3*j12) / (j0*j12 - j12*j12);
	double t23b = (-x1 + a3*j0 - a3*j12) / (j0*j12 - j12*j12);
	double t23 = max(t23a, t23b);

	double t012 = (-a0 + a3 - j12*t23) / j0;

	double a12 = next_a(a0, j0, t012);
	double v12 = next_v(v0, a0, j0, t012);
	double p12 = next_p(p0, v0, a0, j0, t012);
	double p3 = next_p(p12, v12, a12, j12, t23);
	double t34 = dt_from_dp_v(p3, p4, v3);

	double t1 = t0 + t012;
	double t2 = t1;
	double t3 = t2 + t23;
	double t4 = t3 + t34;
	double t5 = t4 + t45;
	double t6 = t5 + t56;
	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v3_from_t12 = next_v(v12, a12, j12, t23);
	double p4_from_t3 = next_p(p3, v3, a3, j3, t34);
	double v3_error = abs(v3_from_t12 - v3);
	double p4_error = abs(p4_from_t3 - p4);

	bool dts_positive = t012 > -1e-6 && t23 > -1e-6 && t34 > -1e-6 && t45 > -1e-6 && t56 > -1e-6 && t67 > -1e-6;
	bool a12_within_limit = abs(a12) < abs(ap);

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v3_error < v_threshold && p4_error < p_threshold && dts_positive && a12_within_limit) {

		t_out[1] = t1;
		t_out[2] = t2;
		t_out[3] = t3;
		t_out[4] = t4;
		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[0] = j0;
		j_out[1] = j12;
		j_out[2] = j12;
		j_out[3] = j3;
		j_out[4] = j4;
		j_out[5] = j5;
		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_7_phase_no_2_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t0 = 0;
	double j0 = jp;
	double a0 = current_state->acc;
	double v0 = current_state->vel;
	double p0 = current_state->pos;
	double j12 = jn;
	double j3 = 0;
	double a3 = 0;
	double v3 = vmax;
	double j4 = jn;
	double a4 = 0;
	double v4 = vmax;
	double j56 = jp;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the right half of the curve

	double x1 = sqrt((j4-j56)*(-j56*a4*a4 + j4*a7*a7 + 2*j4*j56*v4 - 2*j4*j56*v7));
	double t67a = ( x1 + a7*j4 - a7*j56) / (j4*j56 - j56*j56);
	double t67b = (-x1 + a7*j4 - a7*j56) / (j4*j56 - j56*j56);
	double t67 = max(t67a, t67b);
	double t456 = (a7 - a4 - j56*t67) / j4;

	double p56 = last_p(p7, v7, a7, j56, t67);
	double v56 = last_v(v7, a7, j56, t67);
	double a56 = last_a(a7, j56, t67);
	double p4 = last_p(p56, v56, a56, j4, t456);

	// solve the left half of the curve

	double x2 = sqrt((j0-j12) * (-j12*a0*a0 + j0*a3*a3 + 2*j0*j12*v0 - 2*j0*j12*v3));
	double t23a = ( x2 + a3*j0 - a3*j12) / (j0*j12 - j12*j12);
	double t23b = (-x2 + a3*j0 - a3*j12) / (j0*j12 - j12*j12);
	double t23 = max(t23a, t23b);
	double t012 = (-a0 + a3 - j12*t23) / j0;

	double a12 = next_a(a0, j0, t012);
	double v12 = next_v(v0, a0, j0, t012);
	double p12 = next_p(p0, v0, a0, j0, t012);
	double p3 = next_p(p12, v12, a12, j12, t23);

	// solve what's left

	double t34 = dt_from_dp_v(p3, p4, v3);

	double t1 = t0 + t012;
	double t2 = t1;
	double t3 = t2 + t23;
	double t4 = t3 + t34;
	double t5 = t4 + t456;
	double t6 = t5;
	double t7 = t6 + t67;

	// check this against the velocity and position values to see if the solution is valid

	double v3_from_t12 = next_v(v12, a12, j12, t23);
	double v3_from_t56 = last_v(v56, a56, j4, t456);
	double v3_12_error = abs(v3_from_t12 - v3);
	double v3_56_error = abs(v3_from_t56 - v3);

	bool dts_positive = t012 > -1e-6 && t23 > -1e-6 && t34 > -1e-6 && t456 > -1e-6 && t67 > -1e-6;
	bool a12_within_limit = abs(a12) < abs(ap);
	bool a56_within_limit = abs(a56) < abs(an);

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (v3_12_error < v_threshold && v3_56_error < v_threshold && dts_positive && a12_within_limit && a56_within_limit) {

		t_out[1] = t1;
		t_out[2] = t2;
		t_out[3] = t3;
		t_out[4] = t4;
		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[0] = j0;
		j_out[1] = j12;
		j_out[2] = j12;
		j_out[3] = j3;
		j_out[4] = j4;
		j_out[5] = j56;
		j_out[6] = j56;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_7_phase_no_4(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t0 = 0;
	double j0 = jp;
	double a0 = current_state->acc;
	double v0 = current_state->vel;
	double p0 = current_state->pos;
	double j1 = 0;
	double a1 = ap;
	double j234 = jn;
	double a2 = ap;
	double j5 = 0;
	double a5 = an;
	double j6 = jp;
	double a6 = an;
	double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p7 = target_state->pos;

	// solve the unknowns

	double t67 = dt_from_da_j(a6, a7, j6);
	double v6 = last_v(v7, a7, j6, t67);
	double p6 = last_p(p7, v7, a7, j6, t67);
	double t01 = dt_from_da_j(a0, a1, j0);
	double v1 = next_v(v0, a0, j0, t01);
	double p1 = next_p(p0, v0, a0, j0, t01);
	double t2345 = dt_from_da_j(a2, a5, j234);

	double an_2 = an*an;
	double ap_2 = ap*ap;
	double t2345_2 = t2345*t2345;
	double t2345_3 = t2345*t2345_2;
	double t2345_4 = t2345*t2345_3;
	double v1_2 = v1*v1;
	double v6_2 = v6*v6;
	double j234_2 = j234*j234;

	double x3 = an*j234*t2345_2;
	double x2 = 2*(an*ap-an_2);
	double x1 = 2*sqrt(an_2*v1_2 + ap_2*v6_2 + 2*an*ap_2*p1 - 2*an_2*ap*p1 - 2*an*ap_2*p6 + 2*an_2*ap*p6 - an*ap*v1_2 - an*ap*v6_2 + (1/3.0)*an*ap_2*j234*t2345_3 - (1/3.0)*an_2*ap*j234*t2345_3 + 0.25*an*ap*j234_2*t2345_4);
	double t56a = (2*ap*v6 - 2*an*v6 + x1 + x3) / x2;
	double t56b = (2*ap*v6 - 2*an*v6 - x1 + x3) / x2;
	double t56 = max(t56a, t56b);

	double t12 = -(0.5*(j234*t2345_2) + ap*t2345 + v1 - v6 + an*t56) / ap;

	double t1 = t0 + t01;
	double t2 = t1 + t12;
	double t3 = t2;
	double t4 = t3;
	double t5 = t4 + t2345;
	double t6 = t5 + t56;
	double t7 = t6 + t67;

	double tvmax = -a2/j234;
	double v2 = next_v(v1, a1, j1, t12);
	double vmax_measured = next_v(v2, a2, j234, tvmax);

	// check this against the velocity and position values to see if the solution is valid

	bool dts_positive = t01 > -1e-6 && t12 > -1e-6 && t2345 > -1e-6 && t56 > -1e-6 && t67 > -1e-6;
	bool vmax_within_limit = vmax_measured * sign(vmax) < vmax * sign(vmax);

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

	if (dts_positive && vmax_within_limit) {

		t_out[1] = t1;
		t_out[2] = t2;
		t_out[3] = t3;
		t_out[4] = t4;
		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[0] = j0;
		j_out[1] = j1;
		j_out[2] = j234;
		j_out[3] = j234;
		j_out[4] = j234;
		j_out[5] = j5;
		j_out[6] = j6;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_7_phase_no_4_no_2(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

    // initial conditions

    double t0 = 0;
    double j0 = jp;
    double a0 = current_state->acc;
    double v0 = current_state->vel;
    double p0 = current_state->pos;
    double j1 = jn;
    double j5 = 0;
    double a5 = an;
    double j6 = jp;
    double a6 = an;
    double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
    double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
    double p7 = target_state->pos;

    // solve for time six

    double t67 = (a7 - a6) / j6;
    double v6 = last_v(v7, a7, j6, t67);
    double p6 = last_p(p7, v7, a7, j6, t67);

    // precompute common powers

    double a0_2 = a0*a0;
    double a0_3 = a0_2*a0;
    double a0_4 = a0_3*a0;
    double a6_2 = a6*a6;
    double a6_3 = a6_2*a6;
    double a6_4 = a6_3*a6;
    double an_2 = an*an;
    double an_3 = an_2*an;
    double an_4 = an_3*an;
    double jp_2 = jp*jp;
    double jp_3 = jp_2*jp;
    double jp_4 = jp_3*jp;
    double jp_8 = jp_4*jp_4;
    double jp_12 = jp_8*jp_4;
    double jp_16 = jp_8*jp_8;
    double v0_2 = v0*v0;
    double v0_3 = v0_2*v0;
    double v0_4 = v0_3*v0;
    double v6_2 = v6*v6;
    double v6_3 = v6_2*v6;
    double v6_4 = v6_3*v6;
    double root3 = 1.73205080757;
    double root6 = 2.44948974278;
    double onethird = 0.333333333333;
    double twothird = 0.666666666667;

    // solve the rest of the unknowns

    double x14 = 24*a0_3*jp - 48*a0_2*an*jp + 24*a0*an_2*jp + 48*v0*a0*jp_2 - 48*v0*an*jp_2;
    double x13 = 48*a0*jp_3 - 24*an*jp_3;
    double x12 = 60*a0_2*jp_2 - 72*a0*an*jp_2 + 12*an_2*jp_2 + 24*v0*jp_3;
    double x11 = 3*a0_4 - 8*a0_3*an - an_4 + 6*a0_2*an_2 + 12*jp_2*v0_2 - 12*jp_2*v6_2 + 12*a0_2*jp*v0 + 12*an_2*jp*v0 - 24*an*jp_2*p0 + 24*an*jp_2*p6 - 24*a0*an*jp*v0;
    double x10 = x14/(12*jp_4) + pow(x13,3)/(13824*jp_12) - (x13*x12)/(288*jp_8);
    double x9 = x12/(12*jp_4) - (x13*x13)/(384*jp_8);
    double x8 = x11/(12*jp_4) - pow(x13,4)/(1769482*jp_16) - (x13*x14)/(576*jp_8) + (x13*x13*x12)/(27648*jp_12);
    double x7 = sqrt(27*pow(x10,4) + 4*pow(x9,3)*pow(x10,2) - 16*pow(x9,4)*x8 - 256*pow(x8,3) + 128*x9*x9*x8*x8 - 144*x9*x10*x10*x8);
    double x6 = (x10*x10)/2 + (root3*x7)/18 - (4*x9*x8)/3 + pow(x9,3)/27;
    double x5 = x11/jp_4 - 6*x9*pow(x6,onethird) + 9*pow(x6,twothird) + pow(x9,2) - pow(x13,4)/(147456*jp_16) - (x13*x14)/(48*jp_8) + (x13*x13*x12)/(2304*jp_12);
    double x5r = sqrt(x5);
    double x4 = 6*pow(x6,1.0/6)*pow(x5,1.0/4);
    double x3 = x5r/(6*pow(x6,1.0/6));
    double x2 = sqrt( 3*root6*x10*sqrt(27*x10*x10 + 3*root3*x7 - 72*x9*x8 + 2*x9*x9*x9 ) - 12*x8*x5r - x9*x9*x5r - 12*x9*pow(x6,onethird)*x5r - 9*pow(x6,twothird)*x5r ) / x4;
    double x1 = sqrt( -9*pow(x6,twothird)*x5r - 12*x8*x5r - x9*x9*x5r - 12*x9*pow(x6,onethird)*x5r - 3*root6*x10*sqrt( 27*x10*x10 + 3*root3*x7 - 72*x9*x8 + 2*x9*x9*x9 ) ) / x4;

    double t1x[4];
    t1x[0] = -x3 - x13/(48*jp_4) - x2;
    t1x[1] = -x3 - x13/(48*jp_4) + x2;
    t1x[2] =  x3 - x13/(48*jp_4) - x1;
    t1x[3] =  x3 - x13/(48*jp_4) + x1;
    double t1 = NAN;
    for (int i=0; i<4; i++) if ((!isnan(t1x[i]) && isnan(t1)) || ( !isnan(t1x[i]) && !isnan(t1) && t1x[i] > t1)) t1 = t1x[i];

    double a1 = next_a(a0, j0, t1);
    double v1 = next_v(v0, a0, j0, t1);
    double p1 = next_p(p0, v0, a0, j0, t1);
    double t5 = (a1 - an + jp*t1) / jp;
    double v5 = next_v(v1, a1, j1, t5-t1);
    double p5 = next_p(p1, v1, a1, j1, t5-t1);
    double t6 = (v6 - v5 + an*t5) / an;
    double v6_from_v0 = next_v(v5, a5, j5, t6-t5);
    double p6_from_p0 = next_p(p5, v5, a5, j5, t6-t5);
    double t7 = t6 + t67;

    // check this against the velocity and position values to see if the solution is valid

    bool vs_match = abs(v6_from_v0 - v6) < v_threshold;
    bool ps_match = abs(p6_from_p0 - p6) < p_threshold * (abs(p7 - p0));
    bool ts_not_nan = !isnan(t1) && !isnan(t5) && !isnan(t6) && !isnan(t7);
    bool ts_increase = (t1 - 0) > -0.0001 && (t5 - t1) > -0.0001 && (t6 - t5) > -0.0001 && (t7 - t6) > -0.0001;
    bool a_in_range = a1 < limit_pos.acc * 1.0001 && a1 > limit_neg.acc * 1.0001;
    bool v_in_range = v1 < limit_pos.vel && v1 > limit_neg.vel;

    // if they match, then we return this formatted as a 3-point curve with the first two points at zero.
    // this makes it compatible with solve_position_3_phase, which is the parent method.

    if (vs_match && ps_match && ts_not_nan && ts_increase && a_in_range && v_in_range) {

		t_out[1] = t1;
		t_out[2] = t1;
		t_out[3] = t1;
		t_out[4] = t1;
		t_out[5] = t5;
		t_out[6] = t6;
		t_out[7] = t7;

		j_out[0] = j0;
		j_out[1] = j1;
		j_out[2] = j1;
		j_out[3] = j1;
		j_out[4] = j1;
		j_out[5] = j5;
		j_out[6] = j6;

        return true;
    }

    else {
        return false;
    }
    
}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_7_phase_no_4_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

    // initial conditions

    double j0 = jp;
    double a0 = current_state->acc;
    double v0 = current_state->vel;
    double p0 = current_state->pos;
    double j1 = 0;
    double a1 = ap;
    double j2 = jn;
    double a2 = ap;
    double j6 = jp;
    double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
    double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
    double p7 = target_state->pos;
    double t7 = 0;

    // solve for time one

    double t01 = (a1 - a0) / j0;
    double v1 = next_v(v0, a0, j0, t01);
    double p1 = next_p(p0, v0, a0, j0, t01);

    // precompute common powers

    double a1_2 = a1*a1;
    double a1_3 = a1_2*a1;
    double a1_4 = a1_3*a1;
    double a2_2 = a2*a2;
    double a2_3 = a2_2*a2;
    double a2_4 = a2_3*a2;
    double a7_2 = a7*a7;
    double a7_3 = a7_2*a7;
    double a7_4 = a7_3*a7;
    double ap_2 = ap*ap;
    double ap_3 = ap_2*ap;
    double ap_4 = ap_3*ap;
    double jp_2 = jp*jp;
    double jp_3 = jp_2*jp;
    double jp_4 = jp_3*jp;
    double jp_8 = jp_4*jp_4;
    double jp_12 = jp_8*jp_4;
    double jp_16 = jp_8*jp_8;
    double v1_2 = v1*v1;
    double v1_3 = v1_2*v1;
    double v1_4 = v1_3*v1;
    double v7_2 = v7*v7;
    double v7_3 = v7_2*v7;
    double v7_4 = v7_3*v7;
    double root3 = 1.73205080757;
    double root6 = 2.44948974278;
    double onethird = 0.333333333333;
    double twothird = 0.666666666667;

    // solve the rest of the unknowns

    double x14 = 24*a7_3*jp - 48*a7_2*ap*jp + 24*a7*ap_2*jp + 48*v7*a7*jp_2 - 48*v7*ap*jp_2;
    double x13 = 48*a7*jp_3 - 24*ap*jp_3;
    double x12 = 60*a7_2*jp_2 - 72*a7*ap*jp_2 + 12*ap_2*jp_2 + 24*v7*jp_3;
    double x11 = 3*a7_4 - 8*a7_3*ap - ap_4 + 6*a7_2*ap_2 + 12*jp_2*v7_2 - 12*jp_2*v1_2 + 12*a7_2*jp*v7 + 12*ap_2*jp*v7 - 24*ap*jp_2*p7 + 24*ap*jp_2*p1 - 24*a7*ap*jp*v7;
    double x10 = x14/(12*jp_4) + pow(x13,3)/(13824*jp_12) - (x13*x12)/(288*jp_8);
    double x9 = x12/(12*jp_4) - (x13*x13)/(384*jp_8);
    double x8 = x11/(12*jp_4) - pow(x13,4)/(1769482*jp_16) - (x13*x14)/(576*jp_8) + (x13*x13*x12)/(27648*jp_12);
    double x7 = sqrt(27*pow(x10,4) + 4*pow(x9,3)*pow(x10,2) - 16*pow(x9,4)*x8 - 256*pow(x8,3) + 128*x9*x9*x8*x8 - 144*x9*x10*x10*x8);
    double x6 = (x10*x10)/2 + (root3*x7)/18 - (4*x9*x8)/3 + pow(x9,3)/27;
    double x5 = x11/jp_4 - 6*x9*pow(x6,onethird) + 9*pow(x6,twothird) + pow(x9,2) - pow(x13,4)/(147456*jp_16) - (x13*x14)/(48*jp_8) + (x13*x13*x12)/(2304*jp_12);
    double x5r = sqrt(x5);
    double x4 = 6*pow(x6,1.0/6)*pow(x5,1.0/4);
    double x3 = x5r/(6*pow(x6,1.0/6));
    double x2 = sqrt( 3*root6*x10*sqrt(27*x10*x10 + 3*root3*x7 - 72*x9*x8 + 2*x9*x9*x9 ) - 12*x8*x5r - x9*x9*x5r - 12*x9*pow(x6,onethird)*x5r - 9*pow(x6,twothird)*x5r ) / x4;
    double x1 = sqrt( -9*pow(x6,twothird)*x5r - 12*x8*x5r - x9*x9*x5r - 12*x9*pow(x6,onethird)*x5r - 3*root6*x10*sqrt( 27*x10*x10 + 3*root3*x7 - 72*x9*x8 + 2*x9*x9*x9 ) ) / x4;

    double t6x[4];
    t6x[0] = -x3 - x13/(48*jp_4) - x2;
    t6x[1] = -x3 - x13/(48*jp_4) + x2;
    t6x[2] =  x3 - x13/(48*jp_4) - x1;
    t6x[3] =  x3 - x13/(48*jp_4) + x1;
    double t6 = NAN;
    for (int i=0; i<4; i++) if ((!isnan(t6x[i]) && isnan(t6)) || ( !isnan(t6x[i]) && !isnan(t6) && t6x[i] < t6)) t6 = t6x[i];

    double a6 = last_a(a7, j6, t7-t6);
    double v6 = last_v(v7, a7, j6, t7-t6);
    double p6 = last_p(p7, v7, a7, j6, t7-t6);
    double t2 = (a6 - ap + jp*t6) / jp;
    double v2 = last_v(v6, a6, j2, t6-t2);
    double p2 = last_p(p6, v6, a6, j2, t6-t2);
    double t1 = (v1 - v2 + ap*t2) / ap;
    double v1_from_v7 = last_v(v2, a2, j1, t2-t1);
    double p1_from_p7 = last_p(p2, v2, a2, j1, t2-t1);
    double t0 = t1 - t01;

    // rearrange times since we defined zero to be at the end of the curve, and we want it at the beginning

    double timeshift = -t0;
    t0 += timeshift;
    t1 += timeshift;
    t2 += timeshift;
    t6 += timeshift;
    t7 += timeshift;

    // check this against the velocity and position values to see if the solution is valid

    bool vs_match = abs(v1_from_v7 - v1) < v_threshold;
    bool ps_match = abs(p1_from_p7 - p1) < p_threshold * (abs(p7 - p0)) * 10000;
    bool ts_not_nan = !isnan(t1) && !isnan(t2) && !isnan(t6) && !isnan(t7);
    bool ts_increase = (t1 - 0) > -0.0001 && (t2 - t1) > -0.0001 && (t6 - t2) > -0.0001 && (t7 - t6) > -0.0001;
    bool a_in_range = a6 < limit_pos.acc * 1.0001 && a6 > limit_neg.acc * 1.0001;

    // if they match, then we return this formatted as a 3-point curve with the first two points at zero.
    // this makes it compatible with solve_position_3_phase, which is the parent method.

    if (vs_match && ps_match && ts_not_nan && ts_increase && a_in_range) {

        t_out[0] = 0;
        t_out[1] = t1;
        t_out[2] = t2;
        t_out[3] = t2;
        t_out[4] = t2;
        t_out[5] = t6;
        t_out[6] = t7;
        t_out[7] = t7;

        j_out[0] = j0;
        j_out[1] = j1;
        j_out[2] = j2;
        j_out[3] = j2;
        j_out[4] = j2;
        j_out[5] = j6;
        j_out[6] = 0;

        return true;
    }

    else {
        return false;
    }

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_5_phase_no_4_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

    // initial conditions

    double t2 = 0;
    double j2 = jn;
    double a2 = current_state->acc;
    double v2 = current_state->vel;
    double p2 = current_state->pos;
    double j6 = jp;
    double a7 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
    double v7 = isnan(target_state->vel) ? 0 : target_state->vel;
    double p7 = target_state->pos;

    // solve

    double x2 = a2*0.5 + a7*0.5;
    double x1 = pow(a2 - a7, 2) / (8*jp);
    double a27 = a2-a7;
    double a27_2 = a27*a27;
    double a27_3 = a27_2*a27;
    double jp_2 = jp*jp;
    double n = p2 - p7 - a27_3/(24*jp_2) - a27*(v7-x1+(a27*x2/(2*jp)))/(2*jp) + v2*a27/(2*jp) + a2*a27_2/(8*jp_2) + a27_2*x2/(8*jp_2);
    double d = v2*0.5 + v7*0.5 - x1 + a2*a27/(4*jp) + a27*(a2*0.25+a7*0.25)/(2*jp);
    double t7 = -n/d;

    double t6 = (a2 - a7 + jp * t7) / (2 * jp);

    double a6 = next_a(a2, j2, t6);
    double v6 = next_v(v2, a2, j2, t6);
    double p6 = next_p(p2, v2, a2, j2, t6);

    double a7_from_a2 = next_a(a6, j6, t7-t6);
    double v7_from_v2 = next_v(v6, a6, j6, t7-t6);
    double p7_from_p2 = next_p(p6, v6, a6, j6, t7-t6);

    bool as_match = abs(a7_from_a2 - a7) < a_threshold;
    bool vs_match = abs(v7_from_v2 - v7) < v_threshold * 1e4;
    bool ps_match = abs(p7_from_p2 - p7) < p_threshold * 1e4;

    bool ts_not_nan = !isnan(t6) && !isnan(t7);
    bool ts_increase = t6 - t2 > -0.00001 && t7 - t6 > -0.00001;

    bool a_in_range = a6 < limit_pos.acc * 1.0001 && a6 > limit_neg.acc * 1.0001;

    if (as_match && vs_match && ps_match && ts_not_nan && ts_increase && a_in_range) {

        t_out[0] = 0;
        t_out[1] = 0;
        t_out[2] = t2;
        t_out[3] = t2;
        t_out[4] = t2;
        t_out[5] = t2;
        t_out[6] = t6;
        t_out[7] = t7;

        j_out[0] = 0;
        j_out[1] = 0;
        j_out[2] = j2;
        j_out[3] = j2;
        j_out[4] = j2;
        j_out[5] = j2;
        j_out[6] = j6;

        return true;
    }

    else {
        return false;
    }

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_last_7_phase_no_4_no_2_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]) {

	// initial conditions

	double t0 = 0;
	double a0 = current_state->acc;
	double v0 = current_state->vel;
	double p0 = current_state->pos;
	double a3 = isnan(target_state->acc) ? 0 : target_state->acc; // if the input is NAN, use zero instead.  NAN is a deliberate possibility, not a glitch.
	double v3 = isnan(target_state->vel) ? 0 : target_state->vel;
	double p3 = target_state->pos;

	// precompute powers to clean up math

	double a0_2 = a0 * a0;
	double a0_3 = a0_2 * a0;
	double a0_4 = a0_3 * a0;
	double v0_2 = v0 * v0;
	double v0_3 = v0_2 * v0;
	double v0_4 = v0_3 * v0;
	double p0_2 = p0 * p0;
	double p0_3 = p0_2 * p0;
	double p0_4 = p0_3 * p0;
	double a3_2 = a3 * a3;
	double a3_3 = a3_2 * a3;
	double a3_4 = a3_3 * a3;
	double v3_2 = v3 * v3;
	double v3_3 = v3_2 * v3;
	double v3_4 = v3_3 * v3;
	double p3_2 = p3 * p3;
	double p3_3 = p3_2 * p3;
	double p3_4 = p3_3 * p3;
	double jp_2 = jp * jp;
	double jp_3 = jp_2 * jp;
	double jp_4 = jp_3 * jp;
	double jp_8 = jp_4 * jp_4;
	double jp_12 = jp_8 * jp_4;
	double jp_16 = jp_8 * jp_8;

	// finding t7

	double root3 = 1.73205080757;
	double root6 = 2.44948974278;
    double onethird = 0.33333333333333;
    double twothird = 0.66666666666666;
    double onesixth = 0.16666666666667;

	std::complex<double> x19 = 4*a0_3*jp - 4*a3_3*jp - 96*jp_3*p0 + 96*jp_3*p3 - 96*a0*jp_2*v3 + 96*a3*jp_2*v0 + 12*a0*a3_2*jp - 12*a0_2*a3*jp;
	std::complex<double> x18 = 12*a0*jp_3 - 12*a3*jp_3;
	std::complex<double> x17 = 6*a0_2*jp_2 - 48*jp_3*v3 - 48*jp_3*v0 + 6*a3_2*jp_2 + 36*a0*a3*jp_2;
	std::complex<double> x16 = a0_4 - 4*a0_3*a3 - 4*a0*a3_3 + a3_4 + 6*a0_2*a3_2 + 48*jp_2*v0_2 + 48*jp_2*v3_2 - 96*jp_2*v0*v3 - 96*a0*jp_2*p0 + 96*a0*jp_2*p3 + 96*a3*jp_2*p0 - 96*a3*jp_2*p3;
    std::complex<double> x15 = pow(x18,3)/(216*jp_12) - x19/(3*jp_4) + (x18*x17)/(18*jp_8);
    std::complex<double> x15_2 = x15*x15;
    std::complex<double> x15_3 = x15_2*x15;
    std::complex<double> x15_4 = x15_3*x15;
	std::complex<double> x14 = x17/(3*jp_4) + (x18*x18)/(24*jp_8);
    std::complex<double> x14_2 = x14*x14;
    std::complex<double> x14_3 = x14_2*x14;
    std::complex<double> x14_4 = x14_3*x14;
	std::complex<double> x13 = x16/(3*jp_4) + pow(x18,4)/(6912*jp_16) - (x18*x19)/(36*jp_8) + (x18*x18*x17)/(432*jp_12);
	std::complex<double> x12 = sqrt(256.0*pow(x13,3) + 16.0*x14_4*x13 + 27.0*x15_4 + 128.0*x14_2*pow(x13,2) - 4.0*x14_3*x15_2 - 144.0*x14*x15_2*x13);
    std::complex<double> x11 = root3*x12/18.0 - x14_3/27.0 + x15_2/2.0 - 4.0*x14*x13/3.0;
    std::complex<double> x10 = 9.0*pow(x11,twothird) + x14_2 + 6.0*x14*pow(x11,onethird) - 4.0*x16/jp_4 - pow(x18,4)/(576*jp_16) + x18*x19/(3*jp_8) - pow(x18,2)*x17/(36*jp_12);
    std::complex<double> x10root = sqrt(x10);
    std::complex<double> x9 = x14_2*x10root;
    std::complex<double> x8 = 9.0*pow(x11,twothird)*x10root;
    std::complex<double> x7 = 12.0*x14*pow(x11,onethird)*x10root;
    std::complex<double> x6 = 3*root6*x15*sqrt(3*root3*x12 - 2.0*x14_3 + 27.0*x15_2 - 72.0*x14*x13);
    std::complex<double> x5 = 6.0*pow(x11,onesixth)*pow(x10,0.25);
    std::complex<double> x4 = x18/(12*jp_4);
    std::complex<double> x3 = x10root/(6.0*pow(x11,onesixth));
    std::complex<double> x2 = sqrt(12.0*x13*x10root - x9 - x8 + x7 + x6) / x5;
    std::complex<double> x1 = sqrt(12.0*x13*x10root - x9 - x8 + x7 - x6) / x5;

	std::complex<double> t3x[4];
	t3x[0] = -x4 - x3 - x2;
	t3x[1] = -x4 - x3 + x2;
	t3x[2] = -x4 + x3 - x1;
	t3x[3] = -x4 + x3 + x1;

	double t3 = NAN;
	for (int i=0; i<4; i++) {
		if (abs(t3x[i].imag()) < 0.0001 && (t3x[i].real() > t3 || isnan(t3))) {
			t3 = t3x[i].real();
		}
	}

	double n = v0 - v3 + jp*t3*t3/2.0 + t3*(a3 - jp*t3) + ((0.5*a0+0.5*a3-0.5*jp*t3)*(a0-a3+jp*t3))/(2*jp) - (a0*(a0-a3+jp*t3))/(2*jp);
	double d = a0 - a3 + jp*t3;
	double t2 = -(n / d);

	double t1 = -(a0 - a3 - 2*jp*t2 + jp*t3) / (2*jp);

    // check this against the velocity and position values to see if the solution is valid

	double a1_check = next_a(a0, jp, t1);
	double v1_check = next_v(v0, a0, jp, t1);
	double p1_check = next_p(p0, v0, a0, jp, t1);
	double a2_check = next_a(a1_check, jn, t2-t1);
	double v2_check = next_v(v1_check, a1_check, jn, t2-t1);
	double p2_check = next_p(p1_check, v1_check, a1_check, jn, t2-t1);
	double a3_check = next_a(a2_check, jp, t3-t2);
	double v3_check = next_v(v2_check, a2_check, jp, t3-t2);
	double p3_check = next_p(p2_check, v2_check, a2_check, jp, t3-t2);

	double a3_error = abs(a3_check - a3);
	double p3_error = abs(p3_check - p3);
	double v3_error = abs(v3_check - v3);
	bool a3_error_ok = a3_error < a_threshold;
    bool p3_error_ok = p3_error < p_threshold;
    bool v3_error_ok = v3_error < v_threshold;

	bool a1_within_limit = a1_check > limit_neg.acc * 1.0001 && a1_check < limit_pos.acc * 1.0001;
	bool a2_within_limit = a2_check > limit_neg.acc * 1.0001 && a2_check < limit_pos.acc * 1.0001;

	double t_vmax = -a1_check / jn;
	double vmax_found = next_v(v1_check, a1_check, jn, t_vmax);
	bool vmax_within_limits = vmax_found > limit_neg.vel * 1.0001 && vmax_found < limit_pos.vel * 1.0001;

	bool dts_positive = t1 > -1e-6 && t2 > t1 - 1e-6 && t3 > t2 - 1e-6;

	bool not_nan = !isnan(t1) && !isnan(t2) && !isnan(t3);

	// if they match, then we return this formatted as a 3-point curve with the first two points at zero.
	// this makes it compatible with solve_position_3_phase, which is the parent method.

    bool valid = a3_error_ok && v3_error_ok && p3_error_ok && a1_within_limit && a2_within_limit && vmax_within_limits && dts_positive && not_nan;

	if (valid) {

        t_out[1] = t0;
		t_out[2] = t0;
		t_out[3] = t0;
		t_out[4] = t0;
		t_out[5] = t1;
		t_out[6] = t2;
		t_out[7] = t3;

		j_out[0] = 0;
		j_out[1] = 0;
		j_out[2] = 0;
		j_out[3] = 0;
		j_out[4] = jp;
		j_out[5] = jn;
		j_out[6] = jp;

		return true;
	}

	else {
		return false;
	}

}

bool Motion1DPositionVelocityAccelJerkDouble::solve_position_test(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[])
{
    t_out[1] = 0.05;
    t_out[2] = 0.09;
    t_out[3] = 0.10;
    t_out[4] = 0;
    t_out[5] = 0;
    t_out[6] = 0;
    t_out[7] = 0;

    j_out[0] = jn;
    j_out[1] = jp;
    j_out[2] = jn;
    j_out[3] = 0;
    j_out[4] = 0;
    j_out[5] = 0;
    j_out[6] = 0;

    return true;
}

bool Motion1DPositionVelocityAccelJerkDouble::target_has_changed_since_last_run(motion_state_s* target_state) {
    bool pos_ok = (isnan(target_state->pos) && isnan(last_user_target.pos)) || (target_state->pos == last_user_target.pos);
    bool vel_ok = (isnan(target_state->vel) && isnan(last_user_target.vel)) || (target_state->vel == last_user_target.vel);
    bool acc_ok = (isnan(target_state->acc) && isnan(last_user_target.acc)) || (target_state->acc == last_user_target.acc);

    if (!pos_ok || !vel_ok || !acc_ok) {
		last_user_target = *target_state;
		return true;
	}
	else {
		return false;
	}
}

void Motion1DPositionVelocityAccelJerkDouble::determine_target_mode() {

	// decide if we're in position, velocity or acceleration target mode
	// this is based on:
	//  - the user's input
	//  - if we had a position or velocity target but reached it and switched to accel mode
	//  - if we're going to exceed a position or velocity limit and need to switch modes to respect that limit

	target_mode = none;
	target.pos = NAN;
	target.vel = NAN;
	target.acc = NAN;

	// start by assuming the user's specified target mode

    if (!isnan(user.pos)) {
    	target_mode = position;
    	target.pos = user.pos;
    	target.vel = user.vel;
    	target.acc = user.acc;
    }

    else if (!isnan(user.vel)) {
    	target_mode = velocity;
    	target.vel = user.vel;
    	target.acc = user.acc;
    }

    else if (!isnan(user.acc)) {
    	target_mode = acceleration;
    	target.acc = user.acc;
    }

    // if we're in position or velocity mode, but have arrived at the target, switch to accel mode to honor the final acceleration target.
    // note this is stateful behavior and reset each time new targets are set.
    // if the user is operating in this mode, they cannot send a continuous stream of target info.

    if (arrived_at_velocity_target || arrived_at_position_target) {
    	target_mode = acceleration;
    	target.acc = user.acc;
    }

    // if we're in acceleration mode but going to overrun a velocity limit, switch to velocity mode at the limit

    motion_state_s next = compute_next_state_from_jerk(&cur, cur.jrk, dt);
    double stopping_jerk = next.acc > 0 ? limit_neg.jrk : limit_pos.jrk;
    double time_to_make_accel_zero = abs(next.acc / stopping_jerk);

    double velocity_if_zero_accel_now = next.vel + next.acc * time_to_make_accel_zero + 0.5 * stopping_jerk * pow(time_to_make_accel_zero, 2);
    bool velocity_in_bounds = cur.vel > limit_neg.vel && cur.vel < limit_pos.vel;

    if (!using_velocity_limit && velocity_in_bounds && target_mode == acceleration) {
		if (velocity_if_zero_accel_now > limit_pos.vel) {
			using_velocity_limit = true;
			velocity_limit = limit_pos.vel;
	    	arrived_at_velocity_target = false;
		}
		if (velocity_if_zero_accel_now < limit_neg.vel) {
			using_velocity_limit = true;
			velocity_limit = limit_neg.vel;
	    	arrived_at_velocity_target = false;
		}
    }

    if (using_velocity_limit) {
    	target_mode = velocity;
    	target.vel = velocity_limit;
    	target.acc = 0;
    }

    // if we're in acceleration or position mode but going to overrun a position limit, switch to position mode at the limit.
    // this overrides the velocity limit

//    double time_to_make_vel_zero = 0;
//    double position_if_zero_vel_now = 0;
//    motion_state_s vzerostate;
//    vzerostate.acc = 0;
//    vzerostate.vel = 0;
//    get_jerk_for_velocity_mode(&next, &vzerostate, &time_to_make_vel_zero, &position_if_zero_vel_now);
//    bool position_in_bounds = cur.pos > limit_neg.pos && cur.pos < limit_pos.pos;
//
//    if (!using_position_limit && position_in_bounds && (target_mode == acceleration || target_mode == velocity)) {
//    	if (position_if_zero_vel_now > limit_pos.pos) {
//			using_position_limit = true;
//			position_limit = limit_pos.pos;
//	    	arrived_at_position_target = false;
//	    	arrived_at_velocity_target = false;
//		}
//		if (position_if_zero_vel_now < limit_neg.pos) {
//			using_position_limit = true;
//			position_limit = limit_neg.pos;
//	    	arrived_at_position_target = false;
//	    	arrived_at_velocity_target = false;
//		}
//    }
//
//    if (using_position_limit) {
//    	target_mode = velocity;
//    	target.pos = position_limit;
//    	target.vel = 0;
//    	target.acc = 0;
//    }

}

bool Motion1DPositionVelocityAccelJerkDouble::check_input_and_config_errors() {

	bool error_found = false;

	error_code.bits.invalid_dt = dt == 0 || isnan(dt);
	if (error_code.bits.invalid_dt) error_found = true;

	error_code.bits.limit_jrk_nan = isnan(limit_neg.jrk) || isnan(limit_pos.jrk);
	if (error_code.bits.limit_jrk_nan) error_found = true;

	error_code.bits.limit_acc_nan = isnan(limit_neg.acc) || isnan(limit_pos.acc);
	if (error_code.bits.limit_acc_nan) error_found = true;

	error_code.bits.limit_vel_nan = isnan(limit_neg.vel) || isnan(limit_pos.vel);
	if (error_code.bits.limit_vel_nan) error_found = true;

	error_code.bits.limit_pos_nan = isnan(limit_neg.pos) || isnan(limit_pos.pos);
	if (error_code.bits.limit_pos_nan) error_found = true;

	error_code.bits.limit_jrk_polarity = !isnan(limit_neg.jrk) && !isnan(limit_pos.jrk) && limit_neg.jrk >= limit_pos.jrk;
	if (error_code.bits.limit_jrk_polarity) error_found = true;

	error_code.bits.limit_acc_polarity = !isnan(limit_neg.acc) && !isnan(limit_pos.acc) && limit_neg.acc > limit_pos.acc;
	if (error_code.bits.limit_acc_polarity) error_found = true;

	error_code.bits.limit_vel_polarity = !isnan(limit_neg.vel) && !isnan(limit_pos.vel) && limit_neg.vel > limit_pos.vel;
	if (error_code.bits.limit_vel_polarity) error_found = true;

	error_code.bits.limit_pos_polarity = !isnan(limit_neg.pos) && !isnan(limit_pos.pos) && limit_neg.pos > limit_pos.pos;
	if (error_code.bits.limit_pos_polarity) error_found = true;

	error_code.bits.no_tgt = isnan(user.acc) && isnan(user.vel) && isnan(user.pos);
	if (error_code.bits.no_tgt) error_found = true;

	return !error_found;
}

bool Motion1DPositionVelocityAccelJerkDouble::check_output_errors() {

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


