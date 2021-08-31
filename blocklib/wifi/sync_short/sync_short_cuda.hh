#pragma once

#include <gnuradio/wifi/sync_short.hh>
#include <pmt/pmt.h>

#include <cuComplex.h>
#include <cuda.h>
#include <cuda_runtime.h>


namespace gr {
namespace wifi {

class sync_short_cuda : public sync_short
{
public:
    sync_short_cuda(const block_args& args);
    virtual work_return_code_t work(std::vector<block_work_input>& work_input,
                                    std::vector<block_work_output>& work_output) override;

    void insert_tag(uint64_t item,
                    double freq_offset,
                    uint64_t input_item,
                    block_work_output& work_output)
    {
        // mylog(boost::format("frame start at in: %2% out: %1%") % item % input_item);

        const pmt::pmt_t key = pmt::string_to_symbol("wifi_start");
        const pmt::pmt_t value = pmt::from_double(freq_offset);
        const pmt::pmt_t srcid = pmt::string_to_symbol(name());
        work_output.add_tag(item, key, value, srcid);
    }

private:
    float d_threshold;
    int d_min_plateau;
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

	const bool d_log;
	const bool d_debug;
};

} // namespace wifi
} // namespace gr