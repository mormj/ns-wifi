#include <cuComplex.h>
#include <cuda.h>
#include <cuda_runtime.h>

__global__ void bpsk_decision_maker(cuFloatComplex *in, uint8_t *out,
                                    int n) {

  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    out[i] = in[i].x > 0;
  }
}

void exec_bpsk_decision_maker(cuFloatComplex *in, uint8_t *out, int n,
                              int grid_size, int block_size,
                              cudaStream_t stream) {
  bpsk_decision_maker<<<grid_size, block_size, 0, stream>>>(in, out, n);
}
