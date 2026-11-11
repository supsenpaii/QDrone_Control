#ifndef QDRONE_RY_PI_CORE_H
#define QDRONE_RY_PI_CORE_H
/* Yaw-only integral controller. Legacy array sizes preserve model/log ABI.
   Roll state slots 0/2 and roll/pitch torque outputs are always zero. */
#ifdef __cplusplus
extern "C" {
#endif
void qdrone_ry_core_step(double state[4], const double error[3],
    const double angles[3], const double gyro[3], const double residual[3],
    double arm, double throttle, double mode, const double enable[2],
    const double cfg[8], double reset, double dt, double torque[3], double diag[10]);
/* invMixer is column-major (MATLAB layout); output = achieved-requested torque. */
void qdrone_ry_core_aw(const double rawMotor[4], const double rawTorque[3],
    const double limitedTorque[3], const double invMixer[16], double residual[3]);
#ifdef __cplusplus
}
#endif
#endif
