#include "decode_packetized_cpu.hh"

#include <boost/crc.hpp>
#include "utils.h"
#include "viterbi_decoder/viterbi_decoder.h"

namespace gr {
namespace wifi {

decode_packetized::sptr decode_packetized::make_cpu(const block_args& args)
{
    return std::make_shared<decode_packetized_cpu>(args);
}

decode_packetized_cpu::decode_packetized_cpu(const decode_packetized::block_args& args)
    : block("decode_packetized_cpu"), decode_packetized(args),
	d_log(args.log),
	d_debug(args.debug),
	d_frame(d_ofdm, 0)
{

    d_equalizer = new equalizer::ls();

    d_bpsk = constellation_bpsk::make();
    d_qpsk = constellation_qpsk::make();
    d_16qam = constellation_16qam::make();
    d_64qam = constellation_64qam::make();
}

std::shared_ptr<block> decode_packetized_cpu::clone() const
{
	return std::make_shared<decode_packetized_cpu>(block_args{d_log, d_debug});
}


work_return_code_t decode_packetized_cpu::work(std::vector<block_work_input>& work_input,
                                             std::vector<block_work_output>& work_output)
{

	return work_return_code_t::WORK_OK;
}


void decode_packetized_cpu::handle_msg_pdus(pmt::pmt_t msg)
{
	// std::cout << "got msg" << std::endl;

    pmt::pmt_t meta(pmt::car(msg));
    pmt::pmt_t data(pmt::cdr(msg));

    // size_t num_pdu_samples = pmt::length(data);
    size_t len_bytes(0);

    const gr_complex *samples =
        (const gr_complex *)pmt::c32vector_elements(data, len_bytes);

    if (!samples) {
      std::cout << "ERROR: Invalid input type - must be a PMT vector"
                << std::endl;
    //   return pmt::PMT_NIL;
    }

    d_frame_bytes = pmt::to_uint64(
        pmt::dict_ref(meta, pmt::mp("frame_bytes"), pmt::PMT_NIL));
    d_frame_symbols = pmt::to_uint64(
        pmt::dict_ref(meta, pmt::mp("frame_symbols"), pmt::PMT_NIL));
    d_frame_encoding =
        pmt::to_uint64(pmt::dict_ref(meta, pmt::mp("encoding"), pmt::PMT_NIL));
    d_bw = pmt::to_double(pmt::dict_ref(meta, pmt::mp("bw"), pmt::PMT_NIL));
    d_freq = pmt::to_double(
        pmt::dict_ref(meta, pmt::mp("freq"), pmt::from_double(2412000000)));
    d_freq_offset = pmt::to_double(
        pmt::dict_ref(meta, pmt::mp("freq_offset"), pmt::from_double(0.0)));
    // auto pc = pmt::to_uint64(
    //     pmt::dict_ref(meta, pmt::mp("packet_cnt"), pmt::from_uint64(0)));


    switch (d_frame_encoding) {
    case 0:
    case 1:
      d_frame_mod = d_bpsk;
      break;
    case 2:
    case 3:
      d_frame_mod = d_qpsk;
      break;
    case 4:
    case 5:
      d_frame_mod = d_16qam;
      break;
    case 6:
    case 7:
      d_frame_mod = d_64qam;
      break;
    default:
      throw new std::runtime_error("invalid encoding");
    }

    size_t len_h, len_prev;
    const gr_complex *H = pmt::c32vector_elements(
        pmt::dict_ref(meta, pmt::mp("H"), pmt::PMT_NIL), len_h);
    const gr_complex *tmp_prev_pilots = pmt::c32vector_elements(
        pmt::dict_ref(meta, pmt::mp("prev_pilots"), pmt::PMT_NIL), len_prev);
    memcpy(d_prev_pilots, tmp_prev_pilots, 4 * sizeof(gr_complex));
    d_equalizer->set_H(H);

    equalize_frame(samples, d_rx_symbols);


    d_ofdm = ofdm_param((Encoding)d_frame_encoding);
    d_frame = frame_param(d_ofdm, d_frame_bytes);

    // need to equalize and demap the samples to d_rx_bits
    if (!decode(d_rx_bits, d_rx_symbols, d_deinterleaved_bits, out_bytes, d_decoder,
           d_frame, d_ofdm))
           {
                    // FILE *pFile;
                    // char tmp[1024];
                    // sprintf(tmp,"/tmp/decode_%d.dat", this->id());
                    // pFile = fopen(tmp, "a");
                    // fprintf(pFile, "x,%d,%d,%d,%d,%.1f,%.1f,%.6f,%d,", pc, d_frame_encoding, d_frame_bytes, d_frame_symbols, d_bw, d_freq, d_freq_offset, len_bytes);
                    // // for (int i=0; i<64*23; i++)
                    // // {
                    // //     fprintf(pFile, "%.6f+%.6f,", real(samples[i]), imag(samples[i]));
                    // // }
                    // for (int i=0; i<64; i++)
                    // {
                    //     fprintf(pFile, "%.6f+%.6f,", real(H[i]), imag(H[i]));
                    // }
                    // for (int i=0; i<4; i++)
                    // {
                    //     fprintf(pFile, "%.6f+%.6f,", real(d_prev_pilots[i]), imag(d_prev_pilots[i]));
                    // }
                    // fprintf(pFile, "\n");
                    // for (int i=0; i<d_frame_symbols*64; i++)
                    // {
                    //     fprintf(pFile, "%.6f+%.6f,", real(samples[i]), imag(samples[i]));
                    // }
                    // fprintf(pFile, "\n");
                    // for (int i=0; i<d_frame.n_sym*48; i++)
                    // {
                    //     fprintf(pFile, "%d,", d_rx_symbols[i]);
                    // }
                    // fprintf(pFile, "\n");

                    // // fwrite(rx_bits, 1, frame_info.n_sym * 48 , pFile);
                    // fclose(pFile);
           }
           else
           {

                    // FILE *pFile;
                    // char tmp[1024];
                    // sprintf(tmp,"/tmp/decode_%d.dat", this->id());
                    // pFile = fopen(tmp, "a");
                    // fprintf(pFile, "o,%d,%d,%d,%d,%.1f,%.1f,%.6f,%d,", pc, d_frame_encoding, d_frame_bytes, d_frame_symbols, d_bw, d_freq, d_freq_offset, len_bytes);
                    // // for (int i=0; i<64*23; i++)
                    // // {
                    // //     fprintf(pFile, "%.6f+%.6f,", real(samples[i]), imag(samples[i]));
                    // // }
                    // for (int i=0; i<64; i++)
                    // {
                    //     fprintf(pFile, "%.6f+%.6f,", real(H[i]), imag(H[i]));
                    // }
                    // for (int i=0; i<4; i++)
                    // {
                    //     fprintf(pFile, "%.6f+%.6f,", real(d_prev_pilots[i]), imag(d_prev_pilots[i]));
                    // }
                    // fprintf(pFile, "\n");
                    // for (int i=0; i<d_frame_symbols*64; i++)
                    // {
                    //     fprintf(pFile, "%.6f+%.6f,", real(samples[i]), imag(samples[i]));
                    // }
                    // fprintf(pFile, "\n");
                    // for (int i=0; i<d_frame.n_sym*48; i++)
                    // {
                    //     fprintf(pFile, "%d,", d_rx_symbols[i]);
                    // }
                    // fprintf(pFile, "\n");

                    // // fwrite(rx_bits, 1, frame_info.n_sym * 48 , pFile);
                    // fclose(pFile);
           }

    // Insert MAC Decode code here
    // std::cout << "Threadpool got new burst" << std::endl;

    // repackage as pmt and place on output queue
    // pmt::pmt_t vecpmt(pmt::init_c32vector(nproduced, &buffer1[0]));
    // pmt::pmt_t pdu(pmt::cons(pmt::PMT_NIL, vecpmt));
    pmt::pmt_t pdu = pmt::PMT_NIL;

    this->packet_cnt++;
	if (packet_cnt % 1000 == 0)
      std::cout << "decoded: " << packet_cnt << std::endl;

	// send the pdu out the output port

    // return pdu;
}

void decode_packetized_cpu::deinterleave() {

	int n_cbps = d_ofdm.n_cbps;
	int first[n_cbps];
	int second[n_cbps];
	int s = std::max(d_ofdm.n_bpsc / 2, 1);

	for(int j = 0; j < n_cbps; j++) {
		first[j] = s * (j / s) + ((j + int(floor(16.0 * j / n_cbps))) % s);
	}

	for(int i = 0; i < n_cbps; i++) {
		second[i] = 16 * i - (n_cbps - 1) * int(floor(16.0 * i / n_cbps));
	}

	for(int i = 0; i < d_frame.n_sym; i++) {
		for(int k = 0; k < n_cbps; k++) {
			d_deinterleaved_bits[i * n_cbps + second[first[k]]] = d_rx_bits[i * n_cbps + k];
		}
	}
}


void decode_packetized_cpu::descramble (uint8_t *decoded_bits) {

	int state = 0;
	std::memset(out_bytes, 0, d_frame.psdu_size+2);

	for(int i = 0; i < 7; i++) {
		if(decoded_bits[i]) {
			state |= 1 << (6 - i);
		}
	}
	out_bytes[0] = state;

	int feedback;
	int bit;

	for(int i = 7; i < d_frame.psdu_size*8+16; i++) {
		feedback = ((!!(state & 64))) ^ (!!(state & 8));
		bit = feedback ^ (decoded_bits[i] & 0x1);
		out_bytes[i/8] |= bit << (i%8);
		state = ((state << 1) & 0x7e) | feedback;
	}
}

void decode_packetized_cpu::print_output() {

	dout << std::endl;
	dout << "psdu size" << d_frame.psdu_size << std::endl;
	for(int i = 2; i < d_frame.psdu_size+2; i++) {
		dout << std::setfill('0') << std::setw(2) << std::hex << ((unsigned int)out_bytes[i] & 0xFF) << std::dec << " ";
		if(i % 16 == 15) {
			dout << std::endl;
		}
	}
	dout << std::endl;
	for(int i = 2; i < d_frame.psdu_size+2; i++) {
		if((out_bytes[i] > 31) && (out_bytes[i] < 127)) {
			dout << ((char) out_bytes[i]);
		} else {
			dout << ".";
		}
	}
	dout << std::endl;
}

} // namespace wifi
} // namespace gr
