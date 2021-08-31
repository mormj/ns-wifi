#pragma once

#include "equalizer/base.h"
#include "viterbi_decoder/viterbi_decoder.h"
#include <gnuradio/wifi/constellations.hh>
#include <gnuradio/wifi/frame_equalizer.hh>

#include <mutex>

#include <gnuradio/wifi/wifi_types.hh>
#include <cuComplex.h>
#include <cuda.h>
#include <cuda_runtime_api.h>

namespace gr {
namespace wifi {

class frame_equalizer_cuda : public frame_equalizer
{
public:
    frame_equalizer_cuda(const block_args& args);
    virtual work_return_code_t work(std::vector<block_work_input>& work_input,
                                    std::vector<block_work_output>& work_output) override;

    void set_algorithm(Equalizer algo);

private:
    bool parse_signal(uint8_t* signal);
    bool decode_signal_field(uint8_t* rx_bits);
    void deinterleave(uint8_t* rx_bits);

    equalizer::base* d_equalizer = nullptr;
    // gr::thread::mutex d_mutex;
    std::vector<gr::tag_t> tags;
    bool d_debug;
    bool d_log;
    int d_current_symbol;
    viterbi_decoder d_decoder;

    // freq offset
    double d_freq;                      // Hz
    double d_freq_offset_from_synclong; // Hz, estimation from "sync_long" block
    double d_bw;                        // Hz
    double d_er;
    double d_epsilon0;
    gr_complex d_prev_pilots[4];

    int d_frame_bytes;
    int d_frame_symbols;
    int d_frame_encoding;

    uint8_t d_deinterleaved[48];
    gr_complex symbols[48];

    std::shared_ptr<gr::digital::constellation> d_frame_mod;
    constellation_bpsk::sptr d_bpsk;
    constellation_qpsk::sptr d_qpsk;
    constellation_16qam::sptr d_16qam;
    constellation_64qam::sptr d_64qam;

    static const int interleaver_pattern[48];

    cudaStream_t d_stream;
    int d_min_grid_size;
    int d_block_size = 1024;

    float fPOLARITY[127];
    float fLONG[64];
    cuFloatComplex* d_dev_in;
    cuFloatComplex* d_dev_last_symbol;
    float* d_dev_polarity;
    float* d_dev_long_training;
    float* d_dev_beta;
    float* d_dev_er;
    cuFloatComplex* d_dev_H;
    uint8_t* d_dev_out;

    static const int d_max_out_buffer = 1024*1024; // max bytes for output buffer

    enum { WAITING_FOR_TAG, FINISH_LAST_FRAME } d_state = WAITING_FOR_TAG;
};

} // namespace wifi
} // namespace gr