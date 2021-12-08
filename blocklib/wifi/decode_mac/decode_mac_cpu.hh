#pragma once

#include <gnuradio/wifi/decode_mac.hh>
#include <gnuradio/wifi/constellations.hh>
#include "equalizer/base.h"
#include "viterbi_decoder/viterbi_decoder.h"

#include <mutex>

#include <gnuradio/wifi/wifi_types.hh>

namespace gr {
namespace wifi {

class decode_mac_cpu : public decode_mac
{
public:
    decode_mac_cpu(const block_args& args);
    virtual work_return_code_t work(std::vector<block_work_input_sptr>& work_input,
                                    std::vector<block_work_output_sptr>& work_output) override;
private:

	void decode();
	void deinterleave();
	void descramble (uint8_t *decoded_bits);
	void print_output();

	bool d_log;
	bool d_debug;

	ofdm_param d_ofdm = BPSK_1_2;
	frame_param d_frame;
	double d_snr = 0.0;  // dB
	double d_nom_freq = 0.0;  // nominal frequency, Hz
	double d_freq_offset = 0.0;  // frequency offset, Hz
	viterbi_decoder d_decoder;

	uint8_t d_rx_symbols[48 * MAX_SYM];
	uint8_t d_rx_bits[MAX_ENCODED_BITS];
	uint8_t d_deinterleaved_bits[MAX_ENCODED_BITS];
	uint8_t out_bytes[MAX_PSDU_SIZE + 2]; // 2 for signal field

	int copied;
	bool d_frame_complete = true;
};

} // namespace wifi
} // namespace gr