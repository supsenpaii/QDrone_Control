#ifndef QDRONE_FA_CORE_H
#define QDRONE_FA_CORE_H
#define QDRONE_FA_TIMED_STATE_SIZE 1045
#define QDRONE_FA_IMAGE_DELAY_S 0.034
/* R: camera->body FLU, column-major. cfg (13): stage(0 locked/1 flight/2 disarmed shadow),fx,fy,h_ref,
   gain_s,cap_rad,slew_rad_s,filter_s,quality_min,age_max_s,qualify_s,
   stick_deadband_rad,tilt_max_rad. State length 16, diagnostics length 16. */
void qdrone_fa_step(double s[16], const double cmd[3], const double flow[8],
    double frame_time, const double gyro[3], const double attitude[3],
    double arm, double throttle, double base_mode, double request,
    const double cfg[13], const double R[9], double t,
    double output[3], double diag[16]);
/* Stage 3 is a disarmed shadow trial. Stage 4 is an armed trial with runtime
   caps of gain .01 s, angle .5 deg, slew 1 deg/s and tilt 8 deg.
   The first 16 state values retain the legacy layout. Remaining state holds
   timestamped gyro integrals. */
void qdrone_fa_step_timed(double state[QDRONE_FA_TIMED_STATE_SIZE],
    const double cmd[3], const double flow[8], double frame_time,
    const double gyro[3], const double attitude[3],
    double arm, double throttle, double base_mode, double request,
    const double cfg[13], const double R[9], double t,
    double output[3], double diag[16]);
#endif
