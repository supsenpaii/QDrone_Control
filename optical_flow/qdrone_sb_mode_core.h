#ifndef QDRONE_SB_MODE_CORE_H
#define QDRONE_SB_MODE_CORE_H
/* State: initialized,lastTick,lastFresh,seenUp,downSince,upSince.
   SB: UP=-1, MIDDLE=0, DOWN=+1. Outputs mode 0/1/2, diag[6]. */
void qdrone_sb_mode(double state[6],double axis,double fresh,double time,
    double base,double enabled,double *mode,double diag[6]);
#endif
