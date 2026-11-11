#include "mex.h"
#include "qdrone_fa_core.h"
#include <string.h>
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]){
    const size_t n[13]={16,3,8,1,3,3,1,1,1,1,13,9,1};int i;
    size_t state_size;
    if(nrhs!=13||nlhs!=3)mexErrMsgIdAndTxt("QDrone:FA:Args","Expected 13 inputs and 3 outputs.");
    state_size=mxGetNumberOfElements(prhs[0]);
    for(i=0;i<13;i++)if(!mxIsDouble(prhs[i])||mxIsComplex(prhs[i])||mxIsSparse(prhs[i])||
        (i==0?(state_size!=16&&state_size!=QDRONE_FA_TIMED_STATE_SIZE):mxGetNumberOfElements(prhs[i])!=n[i]))
        mexErrMsgIdAndTxt("QDrone:FA:Type","Expected dense real double inputs of the documented sizes.");
    plhs[0]=mxCreateDoubleMatrix(state_size,1,mxREAL);plhs[1]=mxCreateDoubleMatrix(3,1,mxREAL);plhs[2]=mxCreateDoubleMatrix(16,1,mxREAL);
    memcpy(mxGetPr(plhs[0]),mxGetPr(prhs[0]),state_size*sizeof(double));
    if(state_size==QDRONE_FA_TIMED_STATE_SIZE){
        qdrone_fa_step_timed(mxGetPr(plhs[0]),mxGetPr(prhs[1]),mxGetPr(prhs[2]),mxGetScalar(prhs[3]),mxGetPr(prhs[4]),mxGetPr(prhs[5]),
            mxGetScalar(prhs[6]),mxGetScalar(prhs[7]),mxGetScalar(prhs[8]),mxGetScalar(prhs[9]),mxGetPr(prhs[10]),mxGetPr(prhs[11]),mxGetScalar(prhs[12]),
            mxGetPr(plhs[1]),mxGetPr(plhs[2]));
    }else qdrone_fa_step(mxGetPr(plhs[0]),mxGetPr(prhs[1]),mxGetPr(prhs[2]),mxGetScalar(prhs[3]),mxGetPr(prhs[4]),mxGetPr(prhs[5]),
        mxGetScalar(prhs[6]),mxGetScalar(prhs[7]),mxGetScalar(prhs[8]),mxGetScalar(prhs[9]),mxGetPr(prhs[10]),mxGetPr(prhs[11]),mxGetScalar(prhs[12]),
        mxGetPr(plhs[1]),mxGetPr(plhs[2]));
}
