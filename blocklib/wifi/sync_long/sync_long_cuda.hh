#pragma once

#include <gnuradio/wifi/sync_long.hh>

#include <cuComplex.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>

using namespace std;

namespace gr {
namespace wifi {

class sync_long_cuda : public sync_long
{
public:
    sync_long_cuda(const block_args& args);
    virtual work_return_code_t work(std::vector<block_work_input>& work_input,
                                    std::vector<block_work_output>& work_output) override;

private:
	const bool d_log;
	const bool d_debug;

    enum { WAITING_FOR_TAG, FINISH_LAST_FRAME } d_state = WAITING_FOR_TAG;
    unsigned int d_sync_length;
    static const std::vector<gr_complex> LONG;
    int d_fftsize = 512;

    cufftHandle d_plan;
    cufftComplex* d_dev_training_freq;
    cufftComplex* d_dev_in;

    cudaStream_t d_stream;
    int d_min_grid_size;
    int d_block_size;

    std::vector<gr::tag_t> tags;

    int d_ncopied = 0;
    float d_freq_offset = 0;
    float d_freq_offset_short = 0;

    int d_num_syms = 0;
    int ntags = 0;

    size_t packet_cnt = 0;
};

} // namespace wifi
} // namespace gr