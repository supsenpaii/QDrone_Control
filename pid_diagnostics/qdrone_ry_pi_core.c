#include "qdrone_ry_pi_core.h"
#include <math.h>
#include <float.h>
#ifdef _MSC_VER
#define RY_FINITE(x) _finite(x)
#else
#define RY_FINITE(x) isfinite(x)
#endif

static double clamp(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
static int finite_array(const double *x, int n) {
    int k; for (k=0;k<n;k++) if (!RY_FINITE(x[k])) return 0;
    return 1;
}

void qdrone_ry_core_step(double s[4], const double error[3],
    const double angles[3], const double gyro[3], const double residual[3],
    double arm, double throttle, double mode, const double enable[2],
    const double cfg[8], double reset, double dt, double torque[3], double diag[10]) {
    const double rad=0.017453292519943295;
    int k, valid, soft, hard;
    /* Legacy state/log layout is retained; roll integral is permanently removed. */
    s[0]=0;
    s[2]=0;
    for(k=0;k<3;k++) torque[k]=0;
    for(k=0;k<10;k++) diag[k]=0;
    valid=finite_array(s,4) && finite_array(error,3) && finite_array(angles,3)
        && finite_array(gyro,3) && finite_array(residual,3) && finite_array(enable,2)
        && finite_array(cfg,8) && RY_FINITE(arm) && RY_FINITE(throttle)
        && RY_FINITE(mode) && RY_FINITE(reset) && RY_FINITE(dt)
        && dt>0 && dt<=0.005;
    /* Yaw uses cfg slots 1,3,5,7: Ki, cap, slew, back-calculation gain.
       Other slots remain reserved for the existing model/log interface. */
    valid=valid && cfg[1]>=0 && cfg[1]<=0.01
        && cfg[3]>0 && cfg[3]<=0.015
        && cfg[5]>0 && cfg[5]<=0.003
        && cfg[7]>=0 && cfg[7]<=5;
    hard=!valid || fabs(angles[0])>=45*rad || fabs(angles[1])>=45*rad
        || fabs(gyro[0])>=300*rad || fabs(gyro[1])>=300*rad || fabs(gyro[2])>=300*rad;
    if(hard || reset>0.5 || arm<0.5 || throttle<4 || mode>=0.5) {
        for(k=0;k<4;k++) s[k]=0;
        diag[1]=hard ? 5 : 0;
        return;
    }
    soft=fabs(angles[0])>=15*rad || fabs(angles[1])>=15*rad
        || fabs(gyro[0])>=60*rad || fabs(gyro[1])>=60*rad || fabs(gyro[2])>=60*rad;
    {
        const int axis=2, yaw=1;
        const double cap=cfg[3], slew=cfg[5];
        double delta, back=0, proposed;
        int blocked=0;
        if(enable[yaw]<0.5) {s[1]=0;s[3]=0;return;}
        s[1]=clamp(s[1],-cap,cap);
        if(soft) {
            s[3]=fmin(s[3]+dt,1.0);
            /* Brief excursions PAUSE, retaining bounded compensation.
               Sustained excursions shed yaw compensation at 0.015 Nm/s.
               Severe envelope/data faults reset above. */
            if(s[3]>0.2) {
                const double step=0.015*dt;
                s[1]+=clamp(-s[1],-step,step);diag[1]=4;
            } else diag[1]=2;
        } else {
            s[3]=0;
            delta=cfg[1]*error[axis]*dt;
            /* Residual <0 means positive torque was unavailable: do not
               integrate farther into that direction. Opposite error unwinds. */
            blocked=(residual[axis]>1e-8 && delta<0) || (residual[axis]<-1e-8 && delta>0);
            if(blocked) delta=0;
            back=cfg[7]*residual[axis]*dt;
            delta=clamp(delta+back,-slew*dt,slew*dt);
            proposed=s[1]+delta;
            s[1]=clamp(proposed,-cap,cap);
            diag[9]=(proposed!=s[1]) ? 1 : 0;
            diag[1]=diag[9]>0.5 ? 6 : (blocked || fabs(residual[axis])>1e-8 ? 3 : 1);
        }
        torque[axis]=s[1];diag[3]=s[3];diag[5]=(double)blocked;
        diag[7]=back;
    }
}

void qdrone_ry_core_aw(const double m[4], const double raw[3],
    const double lim[3], const double inv[16], double r[3]) {
    double dm[4];int i,j;
    if(!finite_array(m,4) || !finite_array(raw,3) || !finite_array(lim,3) || !finite_array(inv,16)) {
        for(i=0;i<3;i++) r[i]=NAN;
        return;
    }
    for(i=0;i<4;i++) dm[i]=clamp(m[i],0,1)-m[i];
    for(i=0;i<3;i++) {
        r[i]=lim[i]-raw[i];
        for(j=0;j<4;j++) r[i]+=inv[(i+1)+4*j]*dm[j];
    }
}
