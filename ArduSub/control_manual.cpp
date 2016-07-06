/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Sub.h"

// manual_init - initialise manual controller
bool Sub::manual_init(bool ignore_checks)
{
    // Reuse the stabilize_init
    bool success =  stabilize_init(ignore_checks);
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "MANUAL flight mode initialized!");
    return success;
}

// manual_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void Sub::manual_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    motors.set_roll(channel_roll->norm_input());
    motors.set_pitch(channel_pitch->norm_input());
    motors.set_yaw(channel_yaw->norm_input());
    motors.set_throttle(channel_throttle->norm_input());
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}