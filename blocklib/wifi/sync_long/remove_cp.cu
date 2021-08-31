#include <cuComplex.h>
#include <cuda.h>
#include <cuda_runtime.h>

__global__ void
remove_cp(cuFloatComplex* in, cuFloatComplex* out, int symlen, int cplen, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        int sym_idx = i / symlen;
        int samp_idx = i % symlen;

        if (samp_idx >= cplen) {
            out[sym_idx * (symlen-cplen) + samp_idx - cplen] = in[sym_idx * symlen + samp_idx];
        }
    }
}

void exec_remove_cp(cuFloatComplex* in,
                    cuFloatComplex* out,
                    int symlen,
                    int cplen,
                    int n,
                    int grid_size,
                    int block_size,
                    cudaStream_t stream)
{
    remove_cp<<<grid_size, block_size, 0, stream>>>(in, out, symlen, cplen, n);
}

void get_block_and_grid_remove_cp(int* minGrid, int* minBlock)
{
    cudaOccupancyMaxPotentialBlockSize(minGrid, minBlock, remove_cp, 0, 0);
}