#include "qdrone_sb_mode_core.h"
#include <math.h>
#include <float.h>
#ifdef _MSC_VER
#define VALID _finite
#else
#define VALID isfinite
#endif
static void disqualify(double s[6]){s[3]=0;s[4]=s[5]=-1;}
void qdrone_sb_mode(double s[6],double x,double fresh,double t,double base,double en,double *mode,double d[6]){
    int i,status=0,reset=0;double age,held=0;
    *mode=(VALID(base)&&base==1)?1:0;
    for(i=0;i<6;i++){d[i]=0;if(!VALID(s[i]))reset=1;}
    if(reset)for(i=0;i<6;i++)s[i]=0;
    if(!VALID(t)||!VALID(x)||!VALID(fresh)||!VALID(base)||!VALID(en)){
        for(i=0;i<6;i++)s[i]=0;d[0]=6;return;
    }
    if(s[0]<.5){s[0]=1;s[1]=t;s[2]=-1e6;disqualify(s);}
    else if(t<=s[1]||t-s[1]>.02){
        s[0]=0;disqualify(s);d[0]=6;return;
    }
    s[1]=t;if(fresh>.5)s[2]=t;age=t-s[2];
    if(en<.5||base!=0){disqualify(s);status=0;}
    else if(age<0||age>=.25){disqualify(s);status=5;}
    else if(fabs(x+1)<=.15){
        s[4]=-1;if(s[5]<0)s[5]=t;
        if(t-s[5]>=.15)s[3]=1;
        status=s[3]>.5?2:1;
    }
    else if(fabs(x)<=.15){s[4]=s[5]=-1;status=s[3]>.5?2:1;}
    else if(fabs(x-1)<=.15){
        s[5]=-1;
        if(s[3]>.5){
            if(s[4]<0)s[4]=t;
            held=t-s[4];status=3;
            if(held>=.12){*mode=2;status=4;}
        }else{s[4]=-1;status=1;}
    }
    else{disqualify(s);status=6;}
    d[0]=(double)status;d[1]=s[3];d[2]=age;d[3]=x;d[4]=*mode;d[5]=held;
}
