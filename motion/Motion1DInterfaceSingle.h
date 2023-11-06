//
// Created by afabian on 1/6/21.
//

#ifndef MODULES_MOTION1DINTERFACESINGLE_H
#define MODULES_MOTION1DINTERFACESINGLE_H

#include <cmath>
#include <cstdint>
#include "../utils.h"
#include <algorithm>

#define MOTION1DSINGLE_POSITION_THRESHOLD 1e-2
#define MOTION1DSINGLE_VELOCITY_THRESHOLD_SCALE 1e-4
#define MOTION1DSINGLE_ACCELERATION_THRESHOLD_SCALE 1e-4

class Motion1DInterfaceSingle {

public:

    struct motion_state_s {
    	double pos;
    	float vel;
    	float acc;
    	float jrk;
    };

    // configuration

	virtual void configure_position_limits(float _limit_pos_neg, float _limit_pos_pos) = 0;

	void configure_position_limits_int(float _limit_pos_neg, float _limit_pos_pos) {
		limit_neg.pos = _limit_pos_neg;
		limit_pos.pos = _limit_pos_pos;
		if (!std::isnan(user.pos) && !std::isnan(limit_neg.pos) && !std::isnan(limit_pos.pos)) {
			float last_user_pos = user.pos;
			user.pos = minmax((float)last_user_pos, (float)limit_neg.pos, (float)limit_pos.pos);
			error_code.bits.last_pos_tgt_beyond_limits = last_user_pos != user.pos;
		}
	}

	virtual void configure_velocity_limits(float _limit_vel_neg, float _limit_vel_pos) = 0;

	void configure_velocity_limits_int(float _limit_vel_neg, float _limit_vel_pos) {
		limit_neg.vel = _limit_vel_neg;
		limit_pos.vel = _limit_vel_pos;
		if (!std::isnan(user.vel) && !std::isnan(limit_neg.vel) && !std::isnan(limit_pos.vel)) {
			float last_user_vel = user.vel;
			user.vel = minmax(last_user_vel, limit_neg.vel, limit_pos.vel);
			error_code.bits.last_vel_tgt_beyond_limits = last_user_vel != user.vel;
		}
		v_threshold = (limit_pos.vel - limit_neg.vel) * MOTION1DSINGLE_VELOCITY_THRESHOLD_SCALE;
        if (!std::isnan(dt) && !std::isnan(limit_neg.vel) && !std::isnan(limit_pos.vel)) {
            p_threshold = std::max(limit_pos.vel, -limit_neg.vel) * dt * 0.01f;
        }
	}

	virtual void configure_acceleration_limits(float _limit_acc_neg, float _limit_acc_pos) = 0;

	void configure_acceleration_limits_int(float _limit_acc_neg, float _limit_acc_pos) {
		limit_neg.acc = _limit_acc_neg;
		limit_pos.acc = _limit_acc_pos;
		if (!std::isnan(user.acc) && !std::isnan(limit_neg.acc) && !std::isnan(limit_pos.acc)) {
			float last_user_acc = user.acc;
			user.acc = minmax(last_user_acc, limit_neg.acc, limit_pos.acc);
			error_code.bits.last_acc_tgt_beyond_limits = last_user_acc != user.acc;
		}
        a_threshold = (limit_pos.acc - limit_neg.acc) * MOTION1DSINGLE_ACCELERATION_THRESHOLD_SCALE;
    }

	virtual void configure_jerk_limits(float _limit_jrk) = 0;

	void configure_jerk_limits_int(float _limit_jrk) {
		limit_pos.jrk = _limit_jrk;
		limit_neg.jrk = -_limit_jrk;
	}

	virtual void configure_dt(float _dt) = 0;

	void configure_dt_int(float _dt) {
		if (!std::isnan(_dt) && _dt > 0) {
			dt = _dt;
		}
	}

    // input setters

    void jump_to(float _pos = NAN, float _vel = NAN, float _acc = NAN) {
    	// this is not bounds-checked, since it's not a tgt but an actual value
    	cur.pos = std::isnan(_pos) ? 0 : _pos;
    	cur.vel = std::isnan(_vel) ? 0 : _vel;
    	cur.acc = std::isnan(_acc) ? 0 : _acc;
    }

    void set_arrival_time(float _arrival_time) {
	    if (!std::isnan(_arrival_time) && _arrival_time > 0) {
	        user_arrival_time = _arrival_time;
	    }
	}

    void set_target(float _pos = NAN, float _vel = NAN, float _acc = NAN) {
        if (!std::isnan(_pos)) {
        	if (!std::isnan(limit_neg.pos) && !std::isnan(limit_pos.pos)) {
				user.pos = minmax((float)_pos, (float)limit_neg.pos, (float)limit_pos.pos);
				error_code.bits.last_pos_tgt_beyond_limits = user.pos != _pos;
        	}
        	else {
        		user.pos = _pos;
        	}
        }
        else if (!std::isnan(_vel) || !std::isnan(_acc)) {
        	user.pos = NAN;
        }
        if (!std::isnan(_vel)) {
        	if (!std::isnan(limit_neg.vel) && !std::isnan(limit_pos.vel)) {
				user.vel = minmax(_vel, limit_neg.vel, limit_pos.vel);
				error_code.bits.last_vel_tgt_beyond_limits = user.vel != _vel;
        	}
        	else {
        		user.vel = _vel;
        	}
        }
        else {
        	user.vel = NAN;
        }
        if (!std::isnan(_acc)) {
        	if (!std::isnan(limit_neg.acc) && !std::isnan(limit_pos.acc)) {
				user.acc = minmax(_acc, limit_neg.acc, limit_pos.acc);
				error_code.bits.last_acc_tgt_beyond_limits = user.acc != _acc;
        	}
        	else {
        		user.acc = _acc;
        	}
        }
        else {
            user.acc = NAN;
        }
    }

    // main method

    virtual void run_next_timestep() = 0;

    virtual void reset_state() = 0;

    // output getters

    float get_position() {
        return cur.pos;
    }

    float get_velocity() {
        return cur.vel;
    }

    float get_acceleration() {
        return cur.acc;
    }

    float get_jerk() {
        return cur.jrk;
    }

    float get_time_remaining() {
    	return cur_time_remaining;
    }

    float get_debug(int index) {
        return debug[index];
    }

    void get_last_solution(float* t, float* a, float* j) {
        for (int i=0; i<8; i++) {
            t[i] = last_solution_t[i];
            a[i] = last_solution_a[i];
            j[i] = last_solution_j[i];
        }
    }

    motion_state_s get_state_at_time(float t) {
        if (last_solution_var == 'j') {
            return get_state_at_time_from_jerk(t);
        }
        else {
            motion_state_s a = get_state_at_time_from_accel(t);
            if (t > dt) {
                motion_state_s b = get_state_at_time_from_accel(t - dt);
                a.jrk = (a.acc - b.acc) / dt;
            }
            else {
                a.jrk = (a.acc - last_ics.acc) / t;
            }
            return a;
        }
    }

    motion_state_s get_state_at_time_from_jerk(float t) {
        motion_state_s state = last_ics;
        float time_start = 0, time_end = 0;

        // compute and accumulate the system states from the given jerks up to the given t

        for (int stage=0; stage<7; stage++) {
            time_start = max(time_end, last_solution_t[stage]);
            time_end = min(t, max(time_start, last_solution_t[stage + 1]));
            float stage_time = max(0.0f, time_end - time_start);
            float jerk = last_solution_j[stage];
            state = compute_next_state_from_jerk(&state, jerk, stage_time);
            if (time_end >= t) break;
        }

        return state;
    }

    motion_state_s get_state_at_time_from_accel(float t) {
        motion_state_s state = last_ics;
        float time_start = 0, time_end = 0;

        // compute and accumulate the system states from the given accelerations up to the given t

        float accel_sum = 0;

        for (int stage=0; stage<7; stage++) {
            time_start = max(time_end, last_solution_t[stage]);
            time_end = min(t, max(time_start, last_solution_t[stage + 1]));
            float stage_time = max(0.0f, time_end - time_start);
            float accel = last_solution_a[stage];
            accel_sum += accel * stage_time;
            state = compute_next_state_from_accel(&state, accel, stage_time);
            if (time_end >= t) break;
        }

        // overwrite the accel from compute_next_state_from_accel, which will be the last given accel, with the integral of accel over time
        state.acc = accel_sum / time_end;

        // if t is past the end of the planned sequence, assume constant velocity from the last point onwards

        if (time_end < t) {
            float time_remaining = t - time_end;
            state.acc = 0;
            state.jrk = 0;
            state.pos = state.pos + state.vel * time_remaining;
        }

        return state;
    }

    // error reporting

    union error_code_u {
    	struct {
			bool /* bit 00 */ invalid_dt: 1;
			bool /* bit 01 */ no_tgt: 1;
			bool /* bit 02 */ limit_pos_not_impl: 1;
			bool /* bit 03 */ limit_vel_not_impl: 1;
			bool /* bit 04 */ limit_acc_not_impl: 1;
			bool /* bit 05 */ limit_jrk_not_impl: 1;
			bool /* bit 06 */ limit_pos_nan: 1;
			bool /* bit 07 */ limit_vel_nan: 1;
			bool /* bit 08 */ limit_acc_nan: 1;
			bool /* bit 09 */ limit_jrk_nan: 1;
			bool /* bit 10 */ limit_pos_polarity: 1;
			bool /* bit 11 */ limit_vel_polarity: 1;
			bool /* bit 12 */ limit_acc_polarity: 1;
			bool /* bit 13 */ limit_jrk_polarity: 1;
			bool /* bit 14 */ tgt_pos_not_impl: 1;
			bool /* bit 15 */ tgt_vel_not_impl: 1;
			bool /* bit 16 */ tgt_acc_not_impl: 1;
			bool /* bit 17 */ cur_pos_nan: 1;
			bool /* bit 18 */ cur_vel_nan: 1;
			bool /* bit 19 */ cur_acc_nan: 1;
			bool /* bit 20 */ cur_jrk_nan: 1;
			bool /* bit 21 */ last_pos_tgt_beyond_limits: 1;
			bool /* bit 22 */ last_vel_tgt_beyond_limits: 1;
			bool /* bit 23 */ last_acc_tgt_beyond_limits: 1;
			bool /* bit 24 */ tgt_acc_check_failed: 1;
			bool /* bit 25 */ tgt_vel_check_failed: 1;
			bool /* bit 26 */ no_solver_found: 1;
    	} bits;
    	uint64_t value;
    };

    error_code_u get_error_code() {
    	return error_code;
    }

protected:

    motion_state_s compute_next_state_from_vel(motion_state_s* current_state, float vel, float _dt) {
    	motion_state_s next_state;

    	float _dt_2 = _dt * _dt;

        next_state.vel = vel;
        next_state.acc = (next_state.vel - current_state->vel) / dt;
    	next_state.jrk = (next_state.acc - current_state->acc) / _dt_2;
        next_state.pos = current_state->pos + (next_state.vel * _dt);

        return next_state;
    }

    motion_state_s compute_next_state_from_accel(motion_state_s* current_state, float accel, float _dt) {
    	motion_state_s next_state;

    	float _dt_2 = _dt * _dt;

        next_state.acc = accel;
    	next_state.jrk = (next_state.acc - current_state->acc) / dt; // this is wrong; probably need to calculate
        next_state.vel = current_state->vel + (next_state.acc * _dt);
        next_state.pos = current_state->pos + (current_state->vel * _dt) + (0.5f * next_state.acc * _dt_2);

        return next_state;
    }

    motion_state_s compute_next_state_from_jerk(motion_state_s* current_state, float jerk, float _dt) {
    	motion_state_s next_state;

    	float _dt_2 = _dt * _dt;
    	float _dt_3 = _dt_2 * _dt;

    	next_state.jrk = jerk;
        next_state.acc = current_state->acc + (jerk * _dt);
        next_state.vel = current_state->vel + (current_state->acc * _dt) + (0.5f * jerk * _dt_2);
        next_state.pos = current_state->pos + (current_state->vel * _dt) + (0.5f * current_state->acc * _dt_2) + (0.166666667f * jerk * _dt_3);

        return next_state;
    }

    // configuration variables

    motion_state_s limit_neg = {-INFINITY, NAN, NAN, NAN};
    motion_state_s limit_pos = {INFINITY, NAN, NAN, NAN};

    float dt = NAN;

    // user target variables

    motion_state_s user = {NAN, NAN, NAN, NAN};
    float user_arrival_time = 0;

    // internal variables

    motion_state_s last = {0};

    // output variables

    motion_state_s cur = {0};
    float cur_time_remaining = 0;

    // debug and error tracking variables

    float debug[6] = {0};

    error_code_u error_code = { .value = 0 };

    // Use for generating the current path
    // Note that _t and EITHER _j or _a should be set, but not both.
    float last_solution_t[8] = {0};
    float last_solution_j[8] = {0};
    float last_solution_a[8] = {0};
    char last_solution_var = 'j';
    motion_state_s last_ics = {0};

    // threshold variables

    float p_threshold = MOTION1DSINGLE_POSITION_THRESHOLD;
    float v_threshold = 0;
    float a_threshold = 0;

};

#endif //MODULES_MOTION1DINTERFACESINGLE_H
