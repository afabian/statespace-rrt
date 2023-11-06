#ifndef MODULES_MOTION1DPOSITIONVELOCITYACCELSINGLETIMED_H
#define MODULES_MOTION1DPOSITIONVELOCITYACCELSINGLETIMED_H

#include "Motion1DInterfaceSingle.h"
#include "Motion1DPositionVelocityAccelSingle.h"
#include "Motion1DPositionVelocitySingle.h"
#include "Motion1DPositionVelocitySingleTimed.h"

class Motion1DPositionVelocityAccelSingleTimed : public Motion1DInterfaceSingle {

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

    void determine_target_mode();
    bool target_has_changed_since_last_run(motion_state_s* target_state);

    float get_accel_for_velocity_mode(motion_state_s* current_state, motion_state_s* target_state, float arrival_time_target, float* time_remaining);
    motion_state_s get_accel_for_position_mode(motion_state_s* current_state, motion_state_s* target_state, float arrival_time_target, float* time_remaining);
    void update_best_solver(float t[], float a[], float best_t[], float best_a[], int* best_solver, int this_solver);

    bool solve_position_with_velocity_limit(motion_state_s* current_state, motion_state_s* target_state, float target_time_remaining, float vlimit, float t_out[], float a_out[]);
    bool solve_position_no_velocity_limit_2_phase(motion_state_s* current_state, motion_state_s* target_state, float target_time_remaining, float vlimit, float t_out[], float a_out[]);
    bool solve_position_no_velocity_limit_1_phase(motion_state_s* current_state, motion_state_s* target_state, float target_time_remaining, float vlimit, float t_out[], float a_out[]);
    bool solve_position_no_velocity_limit_1_phase_drift(motion_state_s* current_state, motion_state_s* target_state, float target_time_remaining, float vlimit, float t_out[], float a_out[]);

    Motion1DPositionVelocityAccelSingle motion_pva;
    Motion1DPositionVelocitySingle motion_pv;
    Motion1DPositionVelocitySingleTimed motion_pvt;

    bool check_input_and_config_errors();
    bool check_output_errors();

    target_mode_e target_mode = none;
    motion_state_s target = {NAN, NAN, NAN, NAN};
    motion_state_s last_user_target = {NAN, NAN, NAN, NAN};

    bool arrived_at_position_target = false;
    bool arrived_at_position_target_now = false;

    bool using_position_limit = false;
    float position_limit = 0;
};

#endif //MODULES_MOTION1DPOSITIONVELOCITYACCELSINGLETIMED_H
