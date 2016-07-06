/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Sub.h"

/*
 * control_althold.pde - init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Sub::althold_rp_init(bool ignore_checks)
{
	// Reuse the stabilize_init
    bool success = althold_init(ignore_checks);
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "ALT_HOLD_RP flight mode initialized!");
    
    return success;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Sub::althold_rp_run()
{
    althold_run();
    motors.set_yaw(channel_yaw->norm_input());
}
