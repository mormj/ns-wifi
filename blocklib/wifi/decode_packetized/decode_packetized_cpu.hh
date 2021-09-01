#pragma once

#include "equalizer/base.h"
#include "viterbi_decoder/viterbi_decoder.h"
#include <gnuradio/wifi/constellations.hh>
#include <gnuradio/wifi/decode_packetized.hh>

#include <pmt/pmt.h>
#include <mutex>

#include <gnuradio/wifi/wifi_types.hh>


#include "utils.h"
#include <boost/crc.hpp>
#include <vector>

#include "equalizer/base.h"
#include "equalizer/comb.h"
#include "equalizer/lms.h"
#include "equalizer/ls.h"
#include "equalizer/sta.h"

namespace gr {
namespace wifi {

class decode_packetized_cpu : public decode_packetized
{
public:
    decode_packetized_cpu(const block_args& args);
    virtual work_return_code_t work(std::vector<block_work_input>& work_input,
                                    std::vector<block_work_output>& work_output) override;

	int crc_cnt = 0;
private:
    void decode();
    void deinterleave();
    void descramble(uint8_t* decoded_bits);
    void print_output();

    bool d_debug;
    bool d_log;

    ofdm_param d_ofdm = BPSK_1_2;
    frame_param d_frame;
    double d_snr = 0.0;         // dB
    double d_nom_freq = 0.0;    // nominal frequency, Hz
    double d_freq_offset = 0.0; // frequency offset, Hz
    viterbi_decoder d_decoder;

    uint8_t d_rx_symbols[48 * MAX_SYM];
    uint8_t d_rx_bits[MAX_ENCODED_BITS];
    uint8_t d_deinterleaved_bits[MAX_ENCODED_BITS];
    uint8_t out_bytes[MAX_PSDU_SIZE + 2]; // 2 for signal field

    int copied;
    bool d_frame_complete = true;

    int packet_cnt = 0;

    int d_frame_bytes = 0;
    int d_frame_symbols = 0;
    int d_frame_encoding = 0;

    uint8_t d_deinterleaved[48];
    gr_complex symbols[48];

    std::shared_ptr<gr::digital::constellation> d_frame_mod;
    constellation_bpsk::sptr d_bpsk;
    constellation_qpsk::sptr d_qpsk;
    constellation_16qam::sptr d_16qam;
    constellation_64qam::sptr d_64qam;

    equalizer::base* d_equalizer;
    gr_complex d_prev_pilots[4];

    double d_freq;
    double d_bw;


    virtual void handle_msg_pdus(pmt::pmt_t msg);

    const int interleaver_pattern[48] = {
        0, 3, 6, 9,  12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45,
        1, 4, 7, 10, 13, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46,
        2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47
    };

    void decode(uint8_t* rx_bits,
                uint8_t* rx_symbols,
                uint8_t* deinterleaved_bits,
                uint8_t* out_bytes,
                viterbi_decoder& decoder,
                frame_param& frame_info,
                ofdm_param& ofdm_info)
    {

        for (int i = 0; i < frame_info.n_sym * 48; i++) {
            for (int k = 0; k < ofdm_info.n_bpsc; k++) {
                rx_bits[i * ofdm_info.n_bpsc + k] = !!(rx_symbols[i] & (1 << k));
            }
        }

            // FILE *pFile;
            // pFile = fopen("/tmp/ns_rx_bits.dat", "wb");
            // fwrite(rx_bits, 1, frame_info.n_sym * 48 , pFile);
			// fclose(pFile);

        deinterleave(rx_bits, deinterleaved_bits, frame_info.n_sym, ofdm_info);
        uint8_t* decoded = decoder.decode(&ofdm_info, &frame_info, deinterleaved_bits);

        descramble(frame_info.psdu_size, decoded, out_bytes);
        print_output(out_bytes, frame_info.psdu_size);

        // skip service field
        boost::crc_32_type result;
        result.process_bytes(out_bytes + 2, frame_info.psdu_size);
        if (result.checksum() != 558161692) {
            dout << "checksum wrong -- dropping" << std::endl;
            return;
        } else {
            crc_cnt++;
            if (crc_cnt % 1000 == 0) {
                std::cout << "burst_worker:" << crc_cnt << std::endl;
            }
        }

        // mylog(boost::format("encoding: %1% - length: %2% - symbols: %3%") %
            //   ofdm_info.encoding % frame_info.psdu_size % frame_info.n_sym);

        // // create PDU
        // pmt::pmt_t blob = pmt::make_blob(out_bytes + 2, frame_info.psdu_size -
        // 4); d_meta = pmt::dict_add(d_meta, pmt::mp("dlt"),
        //                        pmt::from_long(LINKTYPE_IEEE802_11));

        // message_port_pub(pmt::mp("out"), pmt::cons(d_meta, blob));
    }

    void deinterleave(uint8_t* rx_bits,
                      uint8_t* deinterleaved_bits,
                      size_t n_sym,
                      ofdm_param& ofdm_info)
    {

        int n_cbps = ofdm_info.n_cbps;
        int first[n_cbps];
        int second[n_cbps];
        int s = std::max(ofdm_info.n_bpsc / 2, 1);

        for (int j = 0; j < n_cbps; j++) {
            first[j] = s * (j / s) + ((j + int(floor(16.0 * j / n_cbps))) % s);
        }

        for (int i = 0; i < n_cbps; i++) {
            second[i] = 16 * i - (n_cbps - 1) * int(floor(16.0 * i / n_cbps));
        }

        int count = 0;
        for (int i = 0; i < n_sym; i++) {
            for (int k = 0; k < n_cbps; k++) {
                deinterleaved_bits[i * n_cbps + second[first[k]]] =
                    rx_bits[i * n_cbps + k];
            }
        }
    }

    void descramble(size_t psdu_size, uint8_t* decoded_bits, uint8_t* out_bytes)
    {

        int state = 0;
        std::memset(out_bytes, 0, psdu_size + 2);

        for (int i = 0; i < 7; i++) {
            if (decoded_bits[i]) {
                state |= 1 << (6 - i);
            }
        }
        out_bytes[0] = state;

        int feedback;
        int bit;

        for (int i = 7; i < psdu_size * 8 + 16; i++) {
            feedback = ((!!(state & 64))) ^ (!!(state & 8));
            bit = feedback ^ (decoded_bits[i] & 0x1);
            out_bytes[i / 8] |= bit << (i % 8);
            state = ((state << 1) & 0x7e) | feedback;
        }
    }

    void print_output(uint8_t* out_bytes, size_t psdu_size)
    {

        dout << std::endl;
        dout << "psdu size" << psdu_size << std::endl;
        for (int i = 2; i < psdu_size + 2; i++) {
            dout << std::setfill('0') << std::setw(2) << std::hex
                 << ((unsigned int)out_bytes[i] & 0xFF) << std::dec << " ";
            if (i % 16 == 15) {
                dout << std::endl;
            }
        }
        dout << std::endl;
        for (int i = 2; i < psdu_size + 2; i++) {
            if ((out_bytes[i] > 31) && (out_bytes[i] < 127)) {
                dout << ((char)out_bytes[i]);
            } else {
                dout << ".";
            }
        }
        dout << std::endl;
    }

    void equalize_frame(const gr_complex* symbols, uint8_t* demapped_symbols)
    {

        double d_freq_offset_from_synclong = d_freq_offset * d_bw / (2 * M_PI);
        double d_epsilon0 = d_freq_offset * d_bw / (2 * M_PI * d_freq);
        double d_er = 0.0;

        static gr_complex current_symbol[64];
        static gr_complex tmp_symbols[48];

        for (size_t i = 0; i < d_frame_symbols; i++) {
            size_t d_current_symbol = i + 3;

            std::memcpy(current_symbol, symbols + i * 64, 64 * sizeof(gr_complex));

            // compensate sampling offset
            for (int i = 0; i < 64; i++) {
                current_symbol[i] *=
                    exp(gr_complex(0,
                                   2 * M_PI * d_current_symbol * 80 *
                                       (d_epsilon0 + d_er) * (i - 32) / 64));
            }

            gr_complex p = equalizer::base::POLARITY[(d_current_symbol - 2) % 127];

            double beta;
            if (d_current_symbol < 2) {
                beta = arg(current_symbol[11] - current_symbol[25] + current_symbol[39] +
                           current_symbol[53]);

            } else {
                beta = arg((current_symbol[11] * p) + (current_symbol[39] * p) +
                           (current_symbol[25] * p) + (current_symbol[53] * -p));
            }

            double er = arg((conj(d_prev_pilots[0]) * current_symbol[11] * p) +
                            (conj(d_prev_pilots[1]) * current_symbol[25] * p) +
                            (conj(d_prev_pilots[2]) * current_symbol[39] * p) +
                            (conj(d_prev_pilots[3]) * current_symbol[53] * -p));

            er *= d_bw / (2 * M_PI * d_freq * 80);

            if (d_current_symbol < 2) {
                d_prev_pilots[0] = current_symbol[11];
                d_prev_pilots[1] = -current_symbol[25];
                d_prev_pilots[2] = current_symbol[39];
                d_prev_pilots[3] = current_symbol[53];
            } else {
                d_prev_pilots[0] = current_symbol[11] * p;
                d_prev_pilots[1] = current_symbol[25] * p;
                d_prev_pilots[2] = current_symbol[39] * p;
                d_prev_pilots[3] = current_symbol[53] * -p;
            }

            // compensate residual frequency offset
            for (int i = 0; i < 64; i++) {
                current_symbol[i] *= exp(gr_complex(0, -beta));
            }

            // update estimate of residual frequency offset
            if (d_current_symbol >= 2) {

                double alpha = 0.1;
                d_er = (1 - alpha) * d_er + alpha * er;
            }

            // do equalization
            d_equalizer->equalize(current_symbol,
                                  d_current_symbol,
                                  tmp_symbols,
                                  demapped_symbols + i * 48,
                                  d_frame_mod);
        }

        volatile int x = 7;
    }
};

} // namespace wifi
} // namespace gr