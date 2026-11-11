#include "mex.h"
#include "qdrone_ry_pi_core.h"
#include <string.h>
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {
    static const size_t lengths[12]={4,3,3,3,3,1,1,1,2,8,1,1};
    int i;
    if(nrhs!=12 || nlhs!=3) mexErrMsgIdAndTxt("QDrone:RYPI:Args","Expected 12 inputs and 3 outputs.");
    for(i=0;i<12;i++)
        if(!mxIsDouble(prhs[i]) || mxIsComplex(prhs[i]) || mxIsSparse(prhs[i])
           || mxGetNumberOfElements(prhs[i])!=lengths[i])
            mexErrMsgIdAndTxt("QDrone:RYPI:Type","Inputs must be dense real doubles of the specified size.");
    plhs[0]=mxCreateDoubleMatrix(4,1,mxREAL);
    plhs[1]=mxCreateDoubleMatrix(3,1,mxREAL);
    plhs[2]=mxCreateDoubleMatrix(10,1,mxREAL);
    memcpy(mxGetPr(plhs[0]),mxGetPr(prhs[0]),4*sizeof(double));
    qdrone_ry_core_step(mxGetPr(plhs[0]),mxGetPr(prhs[1]),mxGetPr(prhs[2]),mxGetPr(prhs[3]),
        mxGetPr(prhs[4]),mxGetScalar(prhs[5]),mxGetScalar(prhs[6]),mxGetScalar(prhs[7]),
        mxGetPr(prhs[8]),mxGetPr(prhs[9]),mxGetScalar(prhs[10]),mxGetScalar(prhs[11]),
        mxGetPr(plhs[1]),mxGetPr(plhs[2]));
}
