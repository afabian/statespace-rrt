#ifndef MODULES_MOTION1DPOSITIONVELOCITYACCELJERKSINGLE_H
#define MODULES_MOTION1DPOSITIONVELOCITYACCELJERKSINGLE_H

#include "Motion1DInterfaceSingle.h"
#include "Motion1DPositionVelocitySingle.h"
#include "Motion1DPositionVelocityAccelSingle.h"

class Motion1DPositionVelocityAccelJerkSingle : public Motion1DInterfaceSingle {

public:
    void configure_position_limits(float _limit_pos_neg, float _limit_pos_pos) override;
    void configure_velocity_limits(float _limit_vel_neg, float _limit_vel_pos) override;
    void configure_acceleration_limits(float _limit_acc_neg, float _limit_acc_pos) override;
    void configure_jerk_limits(float _limit_jrk) override;
    void configure_dt(float _dt) override;

    void run_next_timestep() override;
    void reset_state() override;

private:

    enum target_mode_e {
    	none = 0,
		position = 1,
		velocity = 2,
		acceleration = 3,
		jerk = 4
    };

    bool check_input_and_config_errors();
    bool check_output_errors();

    void determine_target_mode();
    bool target_has_changed_since_last_run(motion_state_s* target_state);

    void get_jp_jn_ap_an_vmax_option(int option, float* jp, float* jn, float* ap, float* an, float* vmax);
    float get_jerk_for_accel_mode(motion_state_s* current_state, motion_state_s* target_state, float* time_remaining);
    float get_jerk_for_velocity_mode(motion_state_s* current_state, motion_state_s* target_state, float* time_remaining, float* final_position);
    motion_state_s get_state_for_position_mode(motion_state_s* current_state, motion_state_s* target_state, float* time_remaining);

    bool solve_position_last_1_phase(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_2_phase(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_3_phase(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_4_phase(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_5_phase(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_6_phase(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_7_phase(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_3_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_4_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_5_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_6_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_7_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_7_phase_no_2(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_7_phase_no_2_no_6(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_7_phase_no_4(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_7_phase_no_4_no_2(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_7_phase_no_4_no_6(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_5_phase_no_4_no_6(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_last_7_phase_no_4_no_2_no_6(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);
    bool solve_position_test(motion_state_s* current_state, motion_state_s* target_state, float jp, float jn, float ap, float an, float vmax, float t_out[], float j_out[]);

    void update_best_solver(float t[], float j[], float best_t[], float best_j[], int* best_solver, int this_solver);

    target_mode_e target_mode = none;
    motion_state_s target = {NAN, NAN, NAN, NAN};
    motion_state_s last_user_target = {NAN, NAN, NAN, NAN};

    bool arrived_at_position_target = false;
    bool arrived_at_position_target_now = false;

    bool arrived_at_velocity_target = false;
    bool arrived_at_velocity_target_now = false;

    bool using_position_limit = false;
    float position_limit = 0;

    bool using_velocity_limit = false;
    float velocity_limit = 0;

    Motion1DPositionVelocitySingle motion_pv;
    Motion1DPositionVelocityAccelSingle motion_pva;

    inline float dt_from_da_j(float a_initial, float a_final, float j_const) {
    	return (a_final - a_initial) / j_const;
    }

    inline float dt_from_dv_a(float v_initial, float v_final, float a_const) {
    	return (v_final - v_initial) / a_const;
    }

    inline float dt_from_dp_v(float p_initial, float p_final, float v_const) {
    	return (p_final - p_initial) / v_const;
    }

    inline float next_p(float p0, float v0, float a0, float j0, float dt) {
    	return p0 + v0 * dt + 0.5 * a0 * dt * dt + 0.1666666666666667f * j0 * dt * dt * dt;
    }

    inline float next_v(float v0, float a0, float j0, float dt) {
    	return v0 + a0 * dt + 0.5 * j0 * dt * dt;
    }

    inline float next_a(float a0, float j0, float dt) {
    	return a0 + j0 * dt;
    }

    inline float last_p(float p1, float v1, float a1, float j0, float dt) {
    	return p1 - v1 * dt + 0.5 * a1 * dt * dt - 0.1666666666666667f * j0 * dt * dt * dt;
    }

    inline float last_v(float v1, float a1, float j0, float dt) {
    	return v1 - a1 * dt + 0.5 * j0 * dt * dt;
    }

    inline float last_a(float a1, float j0, float dt) {
    	return a1 - j0 * dt;
    }

    int best_solver = 0;

};

#endif //MODULES_MOTION1DPOSITIONVELOCITYACCELJERKSINGLE_H
