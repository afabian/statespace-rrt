#ifndef MODULES_MOTION1DPOSITIONVELOCITYACCELDOUBLETIMED_H
#define MODULES_MOTION1DPOSITIONVELOCITYACCELDOUBLETIMED_H

#include "Motion1DInterfaceDouble.h"
#include "Motion1DPositionVelocityAccelDouble.h"
#include "Motion1DPositionVelocityDouble.h"
#include "Motion1DPositionVelocityDoubleTimed.h"

class Motion1DPositionVelocityAccelDoubleTimed : public Motion1DInterfaceDouble {

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

    void determine_target_mode();
    bool target_has_changed_since_last_run(motion_state_s* target_state);

    double get_accel_for_velocity_mode(motion_state_s* current_state, motion_state_s* target_state, double arrival_time_target, double* time_remaining);
    motion_state_s get_accel_for_position_mode(motion_state_s* current_state, motion_state_s* target_state, double arrival_time_target, double* time_remaining);
    void update_best_solver(double t[], double a[], double best_t[], double best_a[], int* best_solver, int this_solver);

    bool solve_position_with_velocity_limit(motion_state_s* current_state, motion_state_s* target_state, double target_time_remaining, double vlimit, double t_out[], double a_out[]);
    bool solve_position_no_velocity_limit_2_phase(motion_state_s* current_state, motion_state_s* target_state, double target_time_remaining, double vlimit, double t_out[], double a_out[]);
    bool solve_position_no_velocity_limit_1_phase(motion_state_s* current_state, motion_state_s* target_state, double target_time_remaining, double vlimit, double t_out[], double a_out[]);
    bool solve_position_no_velocity_limit_1_phase_drift(motion_state_s* current_state, motion_state_s* target_state, double target_time_remaining, double vlimit, double t_out[], double a_out[]);

    Motion1DPositionVelocityAccelDouble motion_pva;
    Motion1DPositionVelocityDouble motion_pv;
    Motion1DPositionVelocityDoubleTimed motion_pvt;

    bool check_input_and_config_errors();
    bool check_output_errors();

    target_mode_e target_mode = none;
    motion_state_s target = {NAN, NAN, NAN, NAN};
    motion_state_s last_user_target = {NAN, NAN, NAN, NAN};

    bool arrived_at_position_target = false;
    bool arrived_at_position_target_now = false;

    bool using_position_limit = false;
    double position_limit = 0;
};

#endif //MODULES_MOTION1DPOSITIONVELOCITYACCELDOUBLETIMED_H
