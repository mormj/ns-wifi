#include <cuComplex.h>
#include <cuda.h>
#include <cuda_runtime.h>

__global__ void freq_correction_kernel(cuFloatComplex *in, cuFloatComplex *out,
                                       float freq_offset, float start_idx,
                                       int n) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    // e ix = cos x + i sin x
    float x = -freq_offset * (float)(start_idx + i);
    out[i] = cuCmulf(in[i], make_cuFloatComplex(cos(x), sin(x)));
    // in[i] * expf(make_cuFloatComplex(0, -freq_offset * (start_idx + i)));
  }
}

void exec_freq_correction(cuFloatComplex *in, cuFloatComplex *out,
                          float freq_offset, float start_idx, int n,
                          int grid_size, int block_size, cudaStream_t stream) {
  freq_correction_kernel<<<grid_size, block_size, 0, stream>>>(
      in, out, freq_offset, start_idx, n);
}

void get_block_and_grid_freq_correction(int *minGrid, int *minBlock) {
  cudaOccupancyMaxPotentialBlockSize(minGrid, minBlock, freq_correction_kernel,
                                     0, 0);
}