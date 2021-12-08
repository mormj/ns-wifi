#pragma once

#include <gnuradio/wifi/sync_short.hh>
#include <pmtf/wrap.hpp>
#include <pmtf/string.hpp>
#include <pmtf/scalar.hpp>

#include <cuComplex.h>
#include <cuda.h>
#include <cuda_runtime.h>


namespace gr {
namespace wifi {

class sync_short_cuda : public sync_short
{
public:
    sync_short_cuda(const block_args& args);
    virtual work_return_code_t work(std::vector<block_work_input_sptr>& work_input,
                                    std::vector<block_work_output_sptr>& work_output) override;

    void insert_tag(uint64_t item,
                    double freq_offset,
                    uint64_t input_item,
                    block_work_output_sptr& work_output)
    {
        // mylog(boost::format("frame start at in: %2% out: %1%") % item % input_item);

        const pmtf::wrap key = pmtf::string("wifi_start");
        const pmtf::wrap value = freq_offset;
        const pmtf::wrap srcid = pmtf::string(name());
        work_output->add_tag(item, key, value, srcid);
    }

private:
	const bool d_log;
	const bool d_debug;
    int d_min_plateau;
    float d_threshold;
    uint64_t d_last_tag_location = 0;
    float d_freq_offset;

    size_t packet_cnt = 0;
	
    cuFloatComplex* d_dev_in;
    cuFloatComplex* d_dev_out;
    cudaStream_t d_stream;
    int d_min_grid_size;
    int d_block_size = 1024;

    std::vector<uint8_t> above_threshold;
    std::vector<uint8_t> accum;
	std::vector<float> d_host_cor;
	std::vector<gr_complex> d_host_abs;

    static const int MIN_GAP = 480;
    static const int MAX_SAMPLES = 540 * 80;

    static const int d_max_out_buffer = 1024*1024*2; // max bytes for output buffer


};

} // namespace wifi
} // namespace gr