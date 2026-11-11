# AutoCal log 08/11/2026 14:06:23

Duration 307.660 s, 153824 samples in each stream. Calibration Ready first became 1 at 39.999 s. No Arm requests and no effective Arm samples.

Estimated bias deg/s: X=-0.71834717, Y=-0.18330078, Z=+0.03343506.

At 280–300 s, corrected gyro mean deg/s: X=+0.02872925, Y=+0.01372681, Z=-0.03643799. Estimated roll mean=-0.20706820 deg, pitch mean=-0.25926773 deg. Instantaneous gyro X still varies (std=1.3803 deg/s); small actual rig motion and sensor noise cannot be separated from this log alone.

Conclusion: per-run bias calibration removes the mean offset in this Disarmed recording. Readiness took longer than the ideal 5 s because earlier windows did not pass the stationarity conditions. This log does not validate motor output under thrust, attitude recovery under load, PID gains, or free flight.
