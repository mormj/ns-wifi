#include "pre_sync_cuda.hh"
#include "pre_sync_cuda_gen.hh"

#include <gnuradio/helper_cuda.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuComplex.h>


extern void exec_mov_avg(cuFloatComplex *in, float *mag, cuFloatComplex *out,
                  float *cor, int n, int grid_size, int block_size,
                  cudaStream_t stream);

extern void exec_corr_abs(cuFloatComplex *in, cuFloatComplex *out, float *mag, int n,
                   int grid_size, int block_size, cudaStream_t stream);


namespace gr {
namespace wifi {

pre_sync_cuda::pre_sync_cuda(const pre_sync::block_args& args)
    : block("pre_sync_cuda"), pre_sync(args), d_buffer_size(args.buffer_size), d_window_size(args.window_size)
{
    cudaStreamCreate(&d_stream);

    checkCudaErrors(cudaMalloc((void**)&dev_buf_1, d_buffer_size));
    checkCudaErrors(cudaMalloc((void**)&dev_buf_2, d_buffer_size));
    // checkCudaErrors(cudaMalloc((void**)&dev_buf_3, d_buffer_size));
    // checkCudaErrors(cudaMalloc((void**)&dev_buf_4, d_buffer_size));
    // checkCudaErrors(cudaMalloc((void**)&dev_buf_5, d_buffer_size));
    // checkCudaErrors(cudaMalloc((void**)&dev_buf_6, d_buffer_size));
    // checkCudaErrors(cudaMalloc((void**)&dev_buf_7, d_buffer_size));
    // checkCudaErrors(cudaMalloc((void**)&dev_buf_8, d_buffer_size));
}

work_return_code_t pre_sync_cuda::work(std::vector<block_work_input>& work_input,
                                           std::vector<block_work_output>& work_output)
{
    auto in = work_input[0].items<gr_complex>();
    auto out = work_output[0].items<gr_complex>();
    auto abs = work_output[1].items<gr_complex>();
    auto cor = work_output[2].items<float>();

    auto hist_samps = d_window_size + 16 - 1;

    // input buffer needs to be > noutput_items + hist_samps
    int noutput = std::min(std::min(work_output[0].n_items, work_output[1].n_items),
                           work_output[2].n_items);
    int ninput = work_input[0].n_items;

    // adjust the inputs and outputs
    if (ninput < (int)(noutput + hist_samps + 16)) {
        noutput = ninput - hist_samps - 16;
    } else {
        ninput = noutput + hist_samps + 16;
    }


    auto gridSize = (noutput + 1024 - 1) / 1024;
    exec_corr_abs(((cuFloatComplex *)in), (cuFloatComplex *)dev_buf_1,
                    (float *)dev_buf_2, noutput, gridSize, 1024, d_stream);
    
    exec_mov_avg((cuFloatComplex *)dev_buf_1, (float *)dev_buf_2,
                (cuFloatComplex *)abs, cor, noutput,
                gridSize, 1024, d_stream);

    // memcpy(out, in + 47, noutput_items * sizeof(gr_complex));
    checkCudaErrors(cudaMemcpyAsync(out, in + 47,
                                    sizeof(gr_complex) * noutput,
                                    cudaMemcpyDeviceToDevice, d_stream));

    cudaStreamSynchronize(d_stream);

    // add_item_tag(1, nitems_written(0), pmt::mp("frame"), pmt::from_long(0));





#if 0
    auto tr = work_input[0].nitems_read();
    // After tr == 0, maintain a history of 64 (63 prior samples)
    // auto hist_samps = tr == 0 ? 0 : 63;
    auto hist_samps = d_window_size + 16 - 1;

    // input buffer needs to be > noutput_items + hist_samps
    int noutput = std::min(std::min(work_output[0].n_items, work_output[1].n_items),
                           work_output[2].n_items);
    int ninput = work_input[0].n_items;

    // adjust the inputs and outputs
    if (ninput < (noutput + hist_samps + 16)) {
        noutput = ninput - hist_samps - 16;
    } else {
        ninput = noutput + hist_samps + 16;
    }


    p_conj->launch_default_occupancy({ in + hist_samps + 16 }, { dev_buf_1 }, noutput);
    p_mult->launch_default_occupancy(
        { in + hist_samps, dev_buf_1 }, { dev_buf_2 }, noutput);
    p_cplx_mag_sq->launch_default_occupancy(
        { in + hist_samps - d_window_size }, { dev_buf_3 }, noutput + d_window_size);
    p_ma_ff->launch_default_occupancy({ dev_buf_3 }, { dev_buf_4 }, noutput);
    p_ma_cc->launch_default_occupancy({ dev_buf_2 }, { (void*)out2 }, noutput);
    p_cplx_mag->launch_default_occupancy({ (void*)out2 }, { dev_buf_6 }, noutput);
    p_div->launch_default_occupancy({ dev_buf_4, dev_buf_6 }, { (void*)out3 }, noutput);

    checkCudaErrors(cudaMemcpyAsync(out1,
                                    work_input[0].items() + 16,
                                    sizeof(gr_complex) * noutput,
                                    cudaMemcpyDeviceToDevice,
                                    d_stream));

    cudaStreamSynchronize(d_stream);
#endif
    consume_each(noutput, work_input);
    produce_each(noutput, work_output);
    return work_return_code_t::WORK_OK;
}


} // namespace wifi
} // namespace gr