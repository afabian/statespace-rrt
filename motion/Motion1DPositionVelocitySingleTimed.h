#ifndef MODULES_MOTION1DPOSITIONVELOCITYSINGLETIMED_H
#define MODULES_MOTION1DPOSITIONVELOCITYSINGLETIMED_H

#include "Motion1DInterfaceSingle.h"

class Motion1DPositionVelocitySingleTimed : public Motion1DInterfaceSingle {

public:
    void configure_position_limits(float _limit_pos_neg, float _limit_pos_pos) override;
    void configure_velocity_limits(float _limit_vel_neg, float _limit_vel_pos) override;
    void configure_acceleration_limits(float _limit_acc_neg, float _limit_acc_pos) override;
    void configure_jerk_limits(float _limit_jrk) override;
    void configure_dt(float _dt) override;

    void run_next_timestep() override;
    void reset_state() override;

private:

    float get_vel_for_pos_mode(motion_state_s* current_state, motion_state_s* target_state, float arrival_time);
    float get_time_remaining_for(motion_state_s* current_state, motion_state_s* target_state, float arrival_time);
    bool target_has_changed_since_last_run(motion_state_s* target_state, float target_arrival_time);
    bool check_input_and_config_errors();
    bool check_output_errors();

    bool arrived_at_position_target = false;
    bool using_position_limit = false;
    motion_state_s position_limit = {0};
    motion_state_s last_user_target = {0};
    float last_user_arrival_time = 0;

};

#endif //MODULES_MOTION1DPOSITIONVELOCITYSINGLETIMED_H
