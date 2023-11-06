#ifndef MODULES_MOTION1DPOSITIONVELOCITYACCELJERKDOUBLE_H
#define MODULES_MOTION1DPOSITIONVELOCITYACCELJERKDOUBLE_H

#include "Motion1DInterfaceDouble.h"
#include "Motion1DPositionVelocityDouble.h"
#include "Motion1DPositionVelocityAccelDouble.h"

class Motion1DPositionVelocityAccelJerkDouble : public Motion1DInterfaceDouble {

public:
    void configure_position_limits(double _limit_pos_neg, double _limit_pos_pos) override;
    void configure_velocity_limits(double _limit_vel_neg, double _limit_vel_pos) override;
    void configure_acceleration_limits(double _limit_acc_neg, double _limit_acc_pos) override;
    void configure_jerk_limits(double _limit_jrk) override;
    void configure_dt(double _dt) override;

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

    void get_jp_jn_ap_an_vmax_option(int option, double* jp, double* jn, double* ap, double* an, double* vmax);
    double get_jerk_for_accel_mode(motion_state_s* current_state, motion_state_s* target_state, double* time_remaining);
    double get_jerk_for_velocity_mode(motion_state_s* current_state, motion_state_s* target_state, double* time_remaining, double* final_position);
    motion_state_s get_state_for_position_mode(motion_state_s* current_state, motion_state_s* target_state, double* time_remaining);

    bool solve_position_last_1_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_2_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_3_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_4_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_5_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_6_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_7_phase(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_3_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_4_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_5_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_6_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_7_phase_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_7_phase_no_2(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_7_phase_no_2_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_7_phase_no_4(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_7_phase_no_4_no_2(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_7_phase_no_4_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_5_phase_no_4_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_last_7_phase_no_4_no_2_no_6(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);
    bool solve_position_test(motion_state_s* current_state, motion_state_s* target_state, double jp, double jn, double ap, double an, double vmax, double t_out[], double j_out[]);

    void update_best_solver(double t[], double j[], double best_t[], double best_j[], int* best_solver, int this_solver);

    target_mode_e target_mode = none;
    motion_state_s target = {NAN, NAN, NAN, NAN};
    motion_state_s last_user_target = {NAN, NAN, NAN, NAN};

    bool arrived_at_position_target = false;
    bool arrived_at_position_target_now = false;

    bool arrived_at_velocity_target = false;
    bool arrived_at_velocity_target_now = false;

    bool using_position_limit = false;
    double position_limit = 0;

    bool using_velocity_limit = false;
    double velocity_limit = 0;

    Motion1DPositionVelocityDouble motion_pv;
    Motion1DPositionVelocityAccelDouble motion_pva;

    inline double dt_from_da_j(double a_initial, double a_final, double j_const) {
    	return (a_final - a_initial) / j_const;
    }

    inline double dt_from_dv_a(double v_initial, double v_final, double a_const) {
    	return (v_final - v_initial) / a_const;
    }

    inline double dt_from_dp_v(double p_initial, double p_final, double v_const) {
    	return (p_final - p_initial) / v_const;
    }

    inline double next_p(double p0, double v0, double a0, double j0, double dt) {
    	return p0 + v0 * dt + 0.5 * a0 * dt * dt + 0.1666666666666667 * j0 * dt * dt * dt;
    }

    inline double next_v(double v0, double a0, double j0, double dt) {
    	return v0 + a0 * dt + 0.5 * j0 * dt * dt;
    }

    inline double next_a(double a0, double j0, double dt) {
    	return a0 + j0 * dt;
    }

    inline double last_p(double p1, double v1, double a1, double j0, double dt) {
    	return p1 - v1 * dt + 0.5 * a1 * dt * dt - 0.1666666666666667 * j0 * dt * dt * dt;
    }

    inline double last_v(double v1, double a1, double j0, double dt) {
    	return v1 - a1 * dt + 0.5 * j0 * dt * dt;
    }

    inline double last_a(double a1, double j0, double dt) {
    	return a1 - j0 * dt;
    }

    int best_solver = 0;

};

#endif //MODULES_MOTION1DPOSITIONVELOCITYACCELJERKDOUBLE_H
