// MyKernel.h
#include <cuda_runtime.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <iostream>

#ifndef test_h
#define test_h
__global__ void TestDevice(int* devicearray);
void func(const float* srcptr, float* dstptr, size_t srcstep, const size_t dststep, int cols, int rows);
#endif
