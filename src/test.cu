//.cu code
#include "test.h"
// ...
__global__ void funcKernel(const float* srcptr, float* dstptr, size_t srcstep, const     size_t dststep, int cols, int rows)
{
    int r = blockIdx.y*blockDim.y+threadIdx.y;
    int c = blockIdx.x*blockDim.x+threadIdx.x;
    if(r >= rows || c >= cols)
        return;

    const float* rowsrcptr= (const float *)(((char *)srcptr)+r*srcstep);
    float* rowdstPtr=  (float *) (((char *)dstptr)+r*dststep);

    float val = rowsrcptr[c];
    if((int) val % 90 == 0)
    {
        rowdstPtr[c] = -1 ;
    }
    else
    {
        float acos_val = acos(val);
        rowdstPtr[c] = acos_val;
    }

}

int divUp(int a, int b)
{
    return (a+b-1)/b;

}


void func(const float* srcptr, float* dstptr, size_t srcstep, const size_t dststep, int cols, int rows)
{
    dim3 blDim(32,8);
    dim3 grDim(divUp(cols, blDim.x), divUp(rows,blDim.y));
    std::cout << "calling kernel from func\n";
    funcKernel<<<grDim,blDim>>>(srcptr,dstptr,srcstep,dststep,cols,rows);
    std::cout << "done with kernel call\n";
    cudaDeviceSynchronize();

}


