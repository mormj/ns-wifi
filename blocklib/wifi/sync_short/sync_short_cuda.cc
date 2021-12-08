#include "sync_short_cuda.hh"
#include "sync_short_cuda_gen.hh"

#include <gnuradio/helper_cuda.h>

static const int MIN_GAP = 480;
static const int MAX_SAMPLES = 540 * 80;

extern void exec_freq_correction(cuFloatComplex* in,
                                 cuFloatComplex* out,
                                 float freq_offset,
                                 float start_idx,
                                 int n,
                                 int grid_size,
                                 int block_size,
                                 cudaStream_t stream);

namespace gr {
namespace wifi {

sync_short_cuda::sync_short_cuda(const sync_short::block_args& args)
    : block("sync_short_cuda"),
      sync_short(args),
      d_log(args.log),
      d_debug(args.debug),
      d_min_plateau(args.min_plateau),
      d_threshold(args.threshold)
{

    above_threshold.resize(8192);
    accum.resize(8192);
    d_host_cor.resize(8192);
    d_host_abs.resize(8192);

    cudaStreamCreate(&d_stream);

    set_tag_propagation_policy(tag_propagation_policy_t::TPP_DONT);
}

work_return_code_t sync_short_cuda::work(std::vector<block_work_input_sptr>& work_input,
                                         std::vector<block_work_output_sptr>& work_output)
{
    auto in = work_input[0]->items<gr_complex>();
    auto in_abs = work_input[1]->items<gr_complex>();
    auto  in_cor = work_input[2]->items<float>();
    auto out = work_output[0]->items<gr_complex>();

    int noutput = work_output[0]->n_items;
    int ninput = std::min(std::min(work_input[0]->n_items, work_input[1]->n_items),
                          work_input[2]->n_items);

    noutput = std::min(ninput, noutput);

    int h = d_min_plateau - 1; // index of first real sample
    if (noutput > (int)above_threshold.size()) {
        above_threshold.resize(noutput + h);
        accum.resize(noutput);
        d_host_cor.resize(noutput + h);
        d_host_abs.resize(noutput + h);
    }


    // Copy D2H in_cor to host
    checkCudaErrors(cudaMemcpyAsync(d_host_cor.data(),
                                    in_cor,
                                    sizeof(float) * (noutput + h),
                                    cudaMemcpyDeviceToHost,
                                    d_stream));
    checkCudaErrors(cudaMemcpyAsync(d_host_abs.data(),
                                    in_abs,
                                    sizeof(gr_complex) * (noutput + h),
                                    cudaMemcpyDeviceToHost,
                                    d_stream));
    cudaStreamSynchronize(d_stream);


    for (int i = 0; i < noutput + h; i++) {
        above_threshold[i] = d_host_cor[i] > d_threshold;
    }

    accum[0] = 0;
    for (int j = 0; j < h + 1; j++) {
        if (above_threshold[j]) {
            accum[0]++;
        } else {
            accum[0] = 0;
        }
    }

    uint64_t nread = work_input[0]->nitems_read();
    uint64_t nwritten = work_output[0]->nitems_written();

    for (int i = 1; i < noutput; i++) {
        if (above_threshold[i]) {
            accum[i] = accum[i - 1] + 1;

            if (accum[i] >= d_min_plateau && nread + i - d_last_tag_location > MIN_GAP) {
                d_last_tag_location = nread + i;
                // d_freq_offset = arg(d_host_abs[i]) / 16;
                // There is something wrong with this calculation
                d_freq_offset = 0; // -.00298627;
                // std::cout << "SHORT Frame at " << nwritten + i << std::endl;
                insert_tag(nwritten + i, d_freq_offset, nread + i, work_output[0]);
                packet_cnt++;
                if (packet_cnt % 100 == 0) {
                    std::cout << "sync_short: " << packet_cnt << std::endl;
                }
            }

        } else {
            accum[i] = 0;
        }
    }

    // gr_complex host_in[160];
    // cudaMemcpy(host_in, in+h, 160*sizeof(gr_complex), cudaMemcpyDeviceToHost);

    auto gridSize = (noutput + d_block_size - 1) / d_block_size;
    exec_freq_correction((cuFloatComplex*)in + h,
                         (cuFloatComplex*)out,
                         d_freq_offset,
                         nwritten,
                         noutput,
                         gridSize,
                         d_block_size,
                         d_stream);


    cudaStreamSynchronize(d_stream);


    // memcpy(out_plateau, above_threshold.data(), noutput);
    // memcpy(out_accum, accum.data(), noutput);

    // Tell runtime system how many input items we consumed on
    // each input stream.
    consume_each(noutput, work_input);
    produce_each(noutput, work_output);

    // Tell runtime system how many output items we produced.
    return work_return_code_t::WORK_OK;
}


} // namespace wifi
} // namespace gr