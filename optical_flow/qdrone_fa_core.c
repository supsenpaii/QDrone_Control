#include "qdrone_fa_core.h"
#include <math.h>
#include <float.h>
#include <string.h>
#ifdef _MSC_VER
#define FINITE _finite
#else
#define FINITE isfinite
#endif
static double clip(double v,double lo,double hi){return v<lo?lo:(v>hi?hi:v);}
static int finite_n(const double *v,int n){int i;for(i=0;i<n;i++)if(!FINITE(v[i]))return 0;return 1;}
static void clear_control(double *s){int i;for(i=7;i<=12;i++)s[i]=0;}
static int config_ok(const double *c,const double *R){
    int i,j,k;double det;
    if(!finite_n(c,13)||!finite_n(R,9)||(c[0]!=1&&c[0]!=2&&c[0]!=3&&c[0]!=4)||c[1]<50||c[1]>3000||c[2]<50||c[2]>3000
        ||c[3]<0.3||c[3]>3||c[4]<=0||c[4]>.1||c[5]<=0||c[5]>.05235987756
        ||c[6]<=0||c[6]>.1745329252||c[7]<.03||c[7]>.5||c[8]<.25||c[8]>1
        ||c[9]<.04||c[9]>.12||c[10]<.2||c[10]>2||c[11]<0||c[11]>.03490658504
        ||c[12]<=0||c[12]>.2094395103)return 0;
    /* Stage 4 is the bounded, timestamp-aligned active trial. */
    if(c[0]==4&&(c[4]>.01||c[5]>.00872664626||c[6]>.01745329252||c[12]>.1396263402))return 0;
    for(i=0;i<3;i++)for(j=0;j<3;j++){
        double dot=0;for(k=0;k<3;k++)dot+=R[k+3*i]*R[k+3*j];
        if(fabs(dot-(i==j?1:0))>1e-6)return 0;
    }
    det=R[0]*(R[4]*R[8]-R[7]*R[5])-R[3]*(R[1]*R[8]-R[7]*R[2])+R[6]*(R[1]*R[5]-R[4]*R[2]);
    return det>.999 && R[8]<-.95; /* Down-looking camera only. */
}
static void step_impl(double s[16],const double cmd[3],const double f[8],double ft,
    const double g[3],const double a[3],double arm,double throttle,double mode,double req,
    const double c[13],const double R[9],double t,double out[3],double d[16],
    const double *aligned_omega,int aligned_ready){
    int i,k,newframe,geometry,gate,status=0,shadow;double dt=0,frame_dt=0,age=0;
    double omega[3]={0,0,0},wc[3]={0,0,0},lever[3],lc[3]={0,0,0};
    double compensated[2]={0,0},body[3]={0,0,0},target[2]={0,0};
    for(i=0;i<3;i++)out[i]=cmd[i]; /* Original yaw command is always untouched. */
    for(i=0;i<16;i++)d[i]=0;
    if(!finite_n(s,16)){for(i=0;i<16;i++)s[i]=0;}
    if(!FINITE(t)||!finite_n(cmd,3)||!finite_n(g,3)||!finite_n(a,3)
        ||!FINITE(arm)||!FINITE(throttle)||!FINITE(mode)||!FINITE(req)){
        for(i=0;i<16;i++)s[i]=0;d[0]=8;return;
    }
    if(s[2]>.5){dt=t-s[0];if(dt<=0||dt>.005){
        for(i=0;i<16;i++)s[i]=0;d[0]=8;return;
    }}else{s[2]=1;s[0]=t;}
    s[0]=t;s[3]+=dt;
    for(i=0;i<3;i++)s[4+i]+=g[i]*dt;
    geometry=config_ok(c,R)&&((c[0]!=3&&c[0]!=4)||aligned_omega!=NULL);
    shadow=geometry&&(c[0]==2||c[0]==3);
    gate=geometry && req>.5 && (shadow ? arm<.5 : (arm>.5&&throttle>=4)) && fabs(mode)<.1
        && fabs(cmd[0])<=c[11] && fabs(cmd[1])<=c[11]
        && fabs(a[0])<c[12] && fabs(a[1])<c[12]
        && fabs(g[0])<.698131701 && fabs(g[1])<.698131701 && fabs(g[2])<.698131701;
    newframe=FINITE(ft)&&ft>0&&(s[13]<.5||ft>s[1]+1e-8);
    if(newframe){
        frame_dt=s[13]>.5?ft-s[1]:0;
        if(gate && s[15]>.5 && finite_n(f,8)&&f[3]>.5&&f[2]>=c[8]
            &&f[4]>=.015&&f[4]<=.08&&f[5]>=0&&f[5]<=.01
            &&t>=ft&&t-ft<=c[9]&&frame_dt>=.015&&frame_dt<=.08
            &&fabs(frame_dt-f[4])<.005&&fabs(s[3]-frame_dt)<.005
            &&fabs(f[0])<24&&fabs(f[1])<24&&f[6]>=8&&f[7]>=8
            &&(aligned_omega==NULL||aligned_ready)){
            for(i=0;i<3;i++)omega[i]=aligned_omega!=NULL?aligned_omega[i]:s[4+i]/s[3];
            /* R' maps gyro and lever velocity to camera XYZ. Camera axes:
               right, image-down, optical-forward. Fixed center-flow approximation.
               lever = gyro x [0.10,0,-0.03] m. Scale uses MEASURED nominal
               depth c[3], not a range measurement; valid only near that height. */
            lever[0]=-.03*omega[1];lever[1]=.1*omega[2]+.03*omega[0];lever[2]=-.1*omega[1];
            for(i=0;i<3;i++)for(k=0;k<3;k++){
                wc[i]+=R[k+3*i]*omega[k];lc[i]+=R[k+3*i]*lever[k];
            }
            compensated[0]=f[0]/(c[1]*f[4])+wc[1]+lc[0]/c[3];
            compensated[1]=f[1]/(c[2]*f[4])-wc[0]+lc[1]/c[3];
            for(i=0;i<3;i++)body[i]=-R[i]*compensated[0]-R[i+3]*compensated[1];
            for(i=0;i<2;i++)s[7+i]+=(body[i]-s[7+i])*frame_dt/(c[7]+frame_dt);
            s[11]+=frame_dt;s[12]=1;s[14]=ft;
        }else{clear_control(s);status=4;}
        s[1]=ft;s[13]=1;s[3]=0;for(i=4;i<7;i++)s[i]=0;
    }
    age=s[12]>.5?t-s[14]:1e6;
    if(!geometry){clear_control(s);status=1;}
    else if(req<.5){clear_control(s);status=0;}
    else if(!gate){clear_control(s);status=2;}
    else if(s[12]<.5||age<0||age>c[9]||!FINITE(ft)||!finite_n(f,8)||f[3]<.5
        ||age>c[9]||t<ft||t-ft>c[9]){clear_control(s);status=4;}
    else if(s[11]<c[10]){s[9]=s[10]=0;status=3;}
    else{
        /* FLU: roll produces -Y acceleration; pitch produces +X acceleration. */
        target[0]=clip(c[4]*s[8],-c[5],c[5]);
        target[1]=clip(-c[4]*s[7],-c[5],c[5]);
        for(i=0;i<2;i++){
            s[9+i]+=clip(target[i]-s[9+i],-c[6]*dt,c[6]*dt);
            if(!shadow)out[i]+=s[9+i];
        }
        status=shadow?9:5; /* Shadow computes diagnostics only, never commands. */
    }
    s[15]=gate?1:0;
    d[0]=(double)status;d[1]=req;d[2]=gate?1:0;d[3]=newframe?1:0;
    d[4]=age;d[5]=s[11];d[6]=compensated[0];d[7]=compensated[1];
    d[8]=s[7];d[9]=s[8];d[10]=s[9];d[11]=s[10];
    d[12]=target[0];d[13]=target[1];d[14]=geometry?1:0;d[15]=frame_dt;
}

void qdrone_fa_step(double s[16],const double cmd[3],const double f[8],double ft,
    const double g[3],const double a[3],double arm,double throttle,double mode,double req,
    const double c[13],const double R[9],double t,double out[3],double d[16]){
    step_impl(s,cmd,f,ft,g,a,arm,throttle,mode,req,c,R,t,out,d,NULL,0);
}

/* Ring layout after legacy state: count, newest index, then 256 records of
   [time, integral_x, integral_y, integral_z], followed by previous raw gyro. */
#define HISTORY_CAPACITY 256
#define HISTORY_BASE 18
#define PREVIOUS_GYRO 1042
static int history_integral(const double *s,double time,double result[3]){
    int count=(int)s[16],newest=(int)s[17],offset,k;
    int oldest=(newest-count+1+HISTORY_CAPACITY)%HISTORY_CAPACITY;
    const double *first=s+HISTORY_BASE+4*oldest;
    const double *last=s+HISTORY_BASE+4*newest;
    if(count<2||time<first[0]||time>last[0])return 0;
    /* Search backwards because each image window is close to the newest tick. */
    for(offset=0;offset<count-1;offset++){
        int right=(newest-offset+HISTORY_CAPACITY)%HISTORY_CAPACITY;
        int left=(right-1+HISTORY_CAPACITY)%HISTORY_CAPACITY;
        const double *a=s+HISTORY_BASE+4*left,*b=s+HISTORY_BASE+4*right;
        if(time>=a[0]){
            double fraction=(time-a[0])/(b[0]-a[0]);
            for(k=0;k<3;k++)result[k]=a[k+1]+fraction*(b[k+1]-a[k+1]);
            return 1;
        }
    }
    return 0;
}

void qdrone_fa_step_timed(double s[QDRONE_FA_TIMED_STATE_SIZE],
    const double cmd[3],const double f[8],double ft,const double g[3],const double a[3],
    double arm,double throttle,double mode,double req,const double c[13],const double R[9],
    double t,double out[3],double d[16]){
    int k,count,newest,ready=0;
    double average[3]={0,0,0},begin[3],end[3];
    if(c[0]!=3&&c[0]!=4){
        /* A later switch to the trial must first accumulate its own history. */
        memset(s+16,0,(QDRONE_FA_TIMED_STATE_SIZE-16)*sizeof(double));
        qdrone_fa_step(s,cmd,f,ft,g,a,arm,throttle,mode,req,c,R,t,out,d);
        return;
    }
    if(!finite_n(s,QDRONE_FA_TIMED_STATE_SIZE)||s[16]<0||s[16]>HISTORY_CAPACITY
        ||s[17]<0||s[17]>=HISTORY_CAPACITY||floor(s[16])!=s[16]||floor(s[17])!=s[17]){
        memset(s,0,QDRONE_FA_TIMED_STATE_SIZE*sizeof(double));
    }
    if(!FINITE(t)||!finite_n(g,3)||(s[2]>.5&&(t<=s[0]||t-s[0]>.005))){
        /* Preserve the legacy invalid-input/clock status, then discard history. */
        step_impl(s,cmd,f,ft,g,a,arm,throttle,mode,req,c,R,t,out,d,average,0);
        memset(s,0,QDRONE_FA_TIMED_STATE_SIZE*sizeof(double));
        return;
    }
    count=(int)s[16];newest=(int)s[17];
    if(count==0){
        clear_control(s);
        s[HISTORY_BASE]=t;s[16]=1;s[17]=0;
    }else{
        const double *previous=s+HISTORY_BASE+4*newest;
        double integral[3],dt=t-previous[0];
        for(k=0;k<3;k++)integral[k]=previous[k+1]+.5*(s[PREVIOUS_GYRO+k]+g[k])*dt;
        newest=(newest+1)%HISTORY_CAPACITY;
        s[HISTORY_BASE+4*newest]=t;
        for(k=0;k<3;k++)s[HISTORY_BASE+4*newest+k+1]=integral[k];
        s[16]=count<HISTORY_CAPACITY?count+1:count;s[17]=newest;
    }
    for(k=0;k<3;k++)s[PREVIOUS_GYRO+k]=g[k];
    if(FINITE(ft)&&FINITE(f[4])&&f[4]>=.015&&f[4]<=.08){
        double image_end=ft-QDRONE_FA_IMAGE_DELAY_S;
        ready=history_integral(s,image_end-f[4],begin)&&history_integral(s,image_end,end);
        if(ready)for(k=0;k<3;k++)average[k]=(end[k]-begin[k])/f[4];
    }
    step_impl(s,cmd,f,ft,g,a,arm,throttle,mode,req,c,R,t,out,d,average,ready);
    if(d[0]==8)memset(s,0,QDRONE_FA_TIMED_STATE_SIZE*sizeof(double));
}
