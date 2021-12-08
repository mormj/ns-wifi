#pragma once

#include <gnuradio/wifi/pre_sync.hh>

#include <cusp/complex_to_mag.cuh>
#include <cusp/complex_to_mag_squared.cuh>
#include <cusp/conjugate.cuh>
#include <cusp/convolve.cuh>
#include <cusp/divide.cuh>
#include <cusp/multiply.cuh>

#include <gnuradio/blocks/delay.hh>

namespace gr {
namespace wifi {

class pre_sync_cuda : public pre_sync
{
public:
    pre_sync_cuda(const block_args& args);
    virtual work_return_code_t work(std::vector<block_work_input_sptr>& work_input,
                                    std::vector<block_work_output_sptr>& work_output) override;

private:
    size_t d_buffer_size;
    size_t d_window_size;

    std::shared_ptr<cusp::convolve<float, float>> p_ma_ff;
    std::shared_ptr<cusp::convolve<gr_complex, gr_complex>> p_ma_cc;
    std::shared_ptr<cusp::multiply<gr_complex>> p_mult;
    std::shared_ptr<cusp::divide<float>> p_div;
	std::shared_ptr<cusp::conjugate> p_conj;
    std::shared_ptr<cusp::complex_to_mag> p_cplx_mag;
    std::shared_ptr<cusp::complex_to_mag_squared> p_cplx_mag_sq;

    blocks::delay::sptr p_delay;

    void* dev_buf_1;
    void* dev_buf_2;
    // void* dev_buf_3;
    // void* dev_buf_4;
	// void* dev_buf_5;
	// void* dev_buf_6;
	// void* dev_buf_7;
	// void* dev_buf_8;

	cudaStream_t d_stream;
};

} // namespace wifi
} // namespace gr