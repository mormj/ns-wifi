#include <cuComplex.h>
#include <cuda.h>
#include <cuda_runtime.h>

__global__ void multiply_kernel_ccc(cuFloatComplex *in1, cuFloatComplex *in2,
                                    cuFloatComplex *out, int n) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    float re, im;
    re = in1[i].x * in2[i].x - in1[i].y * in2[i].y;
    im = in1[i].x * in2[i].y + in1[i].y * in2[i].x;
    out[i].x = re;
    out[i].y = im;
  }
}

void exec_multiply_kernel_ccc(cuFloatComplex *in1, cuFloatComplex *in2,
                              cuFloatComplex *out, int n, int grid_size,
                              int block_size, cudaStream_t stream) {
  multiply_kernel_ccc<<<grid_size, block_size, 0, stream>>>(in1, in2, out, n);
}

void get_block_and_grid_multiply(int *minGrid, int *minBlock) {
  cudaOccupancyMaxPotentialBlockSize(minGrid, minBlock, multiply_kernel_ccc, 0,
                                     0);
}