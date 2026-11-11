# AutoCal test 08/11/2026 14:15:18

Read 360511 matched Controller/Motors/GyroCal samples; duration 721.146 s. Ready first true at 31.483 s. No samples with effective Arm while Ready=false.

## Zero-command thrust window 40–44 s

Roll and pitch commands exactly 0. Throttle fixed at 14.84337 N. Measured roll mean -3.92264 deg, pitch +0.98367 deg. Motor channel commands mean: 0.63806, 0.67367, 0.77832, 0.81332. No motor clipping (neither 0 nor 1). Roll torque command mean +0.15275 Nm. Corrected gyro X mean +0.38759 deg/s; this is a powered transient window, not an independent stationary bias measurement.

The persistent attitude error remains after per-run bias calibration. Motor clipping cannot explain this specific interval. This does not independently prove actual thrust balance, sensor-to-body alignment, or rig torque. Existing controller lacks integral action on tracking error, so bounded integral action is a possible next design change after validating feedback direction and physical angle. Do not infer numerical gains from this interval alone.

## Other intervals

At 52.381–56.611 s measured roll spans about -63.21..+63.81 deg while roll command varies -44.55..+24.19 deg. At 293.242–312.832 s measured pitch reaches +69.56 deg and some motor commands reach 1. Later trials also include clipping and changing angle commands. These are not all zero-stick tests and must not be pooled into a stationary throttle-only diagnosis or unrestricted autotune.

No model or PID edits were made during this log analysis.
