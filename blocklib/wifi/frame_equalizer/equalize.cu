#include <cuComplex.h>
#include <cuda.h>
#include <cuda_runtime.h>

__global__ void ls_freq_domain_equalization(cuFloatComplex *in,
                                            cuFloatComplex *out,
                                            cuFloatComplex *H, int n) {

  int i = blockIdx.x * blockDim.x + threadIdx.x;

  int symbol_index = i / 64;
  int sample_index = i % 64;

  if (i < n) {
    if ((sample_index == 11) || (sample_index == 25) || (sample_index == 32) ||
        (sample_index == 39) || (sample_index == 53) || (sample_index < 6) ||
        (sample_index > 58)) {
      return;
    }

    int c = 0;
    if (sample_index < 11) {
      c = sample_index - 6;
    } else if (sample_index < 25) {
      c = sample_index - 7;
    } else if (sample_index < 32) {
        c = sample_index - 8;
    } else if (sample_index < 39) {
      c = sample_index - 9;
    } else {
      c = sample_index - 10;
    }

    out[symbol_index * 48 + c] = cuCdivf(in[i], H[sample_index]);
  }
}

__global__ void ls_freq_domain_chanest(cuFloatComplex *in, float *training_seq,
                                       cuFloatComplex *H) {

  int i = blockIdx.x * blockDim.x + threadIdx.x;

  int symbol_index = i / 64;
  int sample_index = i % 64;

  if (i < 64) {

    // if (sample_index != 32 && sample_index >= 6 && sample_index <= 58) {
      H[sample_index] = cuCaddf(in[sample_index], in[64 + sample_index]);
      H[sample_index] = make_cuFloatComplex(
          H[sample_index].x / (training_seq[sample_index] * 2.0),
          H[sample_index].y / (training_seq[sample_index] * 2.0));
    // }
  }
}

void exec_ls_freq_domain_equalization(cuFloatComplex *in, cuFloatComplex *out,
                                      cuFloatComplex *H, int n, int grid_size,
                                      int block_size, cudaStream_t stream) {
  ls_freq_domain_equalization<<<grid_size, block_size, 0, stream>>>(in, out, H,
                                                                    n);
}

void exec_ls_freq_domain_chanest(cuFloatComplex *in, float *training_seq,
                                 cuFloatComplex *H, int grid_size,
                                 int block_size, cudaStream_t stream) {
  ls_freq_domain_chanest<<<grid_size, block_size, 0, stream>>>(in, training_seq,
                                                               H);
}
