/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Sub::stabilize_rp_init(bool ignore_checks)
{
	// Reuse the stabilize_init
    bool success = stabilize_init(ignore_checks);
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "STABILIZE_RP flight mode initialized!");
    return success;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Sub::stabilize_rp_run()
{
    stabilize_run();
    motors.set_yaw(channel_yaw->norm_input());
}
