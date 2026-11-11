#include "mex.h"
#include "qdrone_sb_mode_core.h"
#include <string.h>
void mexFunction(int nlhs,mxArray *out[],int nrhs,const mxArray *in[]){
    int i;if(nrhs!=6||nlhs!=3)mexErrMsgIdAndTxt("QDrone:SB:Args","Expected 6 inputs and 3 outputs.");
    for(i=0;i<6;i++)if(!mxIsDouble(in[i])||mxIsComplex(in[i])||mxIsSparse(in[i])||mxGetNumberOfElements(in[i])!=(i==0?6:1))
        mexErrMsgIdAndTxt("QDrone:SB:Type","Expected dense real double inputs, state length 6 and 5 scalars.");
    out[0]=mxCreateDoubleMatrix(6,1,mxREAL);out[1]=mxCreateDoubleScalar(0);out[2]=mxCreateDoubleMatrix(6,1,mxREAL);
    memcpy(mxGetPr(out[0]),mxGetPr(in[0]),6*sizeof(double));
    qdrone_sb_mode(mxGetPr(out[0]),mxGetScalar(in[1]),mxGetScalar(in[2]),mxGetScalar(in[3]),mxGetScalar(in[4]),mxGetScalar(in[5]),mxGetPr(out[1]),mxGetPr(out[2]));
}
