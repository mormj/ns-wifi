#include "frame_equalizer_cpu.hh"

#include "equalizer/base.h"
#include "equalizer/comb.h"
#include "equalizer/lms.h"
#include "equalizer/ls.h"
#include "equalizer/sta.h"

#include "utils.h"

#include <pmtf/scalar.hpp>
#include <pmtf/string.hpp>
#include <pmtf/map.hpp>

namespace gr {
namespace wifi {

frame_equalizer::sptr frame_equalizer::make_cpu(const block_args& args)
{
    return std::make_shared<frame_equalizer_cpu>(args);
}

frame_equalizer_cpu::frame_equalizer_cpu(const frame_equalizer::block_args& args)
    : block("frame_equalizer_cpu"), frame_equalizer(args),
	d_log(args.log), d_debug(args.debug),
	d_freq(args.freq), d_bw(args.bw)
{

	d_bpsk = constellation_bpsk::make();
	d_qpsk = constellation_qpsk::make();
	d_16qam = constellation_16qam::make();
	d_64qam = constellation_64qam::make();

	d_frame_mod = d_bpsk;

	set_tag_propagation_policy(tag_propagation_policy_t::TPP_DONT);
	set_algorithm((Equalizer)args.algo);
}


void
frame_equalizer_cpu::set_algorithm(Equalizer algo) {
	
	delete d_equalizer;

	switch(algo) {

	case COMB:
		dout << "Comb" << std::endl;
		d_equalizer = new equalizer::comb();
		break;
	case LS:
		dout << "LS" << std::endl;
		d_equalizer = new equalizer::ls();
		break;
	case LMS:
		dout << "LMS" << std::endl;
		d_equalizer = new equalizer::lms();
		break;
	case STA:
		dout << "STA" << std::endl;
		d_equalizer = new equalizer::sta();
		break;
	default:
		throw std::runtime_error("Algorithm not implemented");
	}
}

work_return_code_t frame_equalizer_cpu::work(std::vector<block_work_input>& work_input,
                                             std::vector<block_work_output>& work_output)
{
    auto in = work_input[0].items<gr_complex>();
    auto out = work_output[0].items<uint8_t>();

    auto ninput_items = work_input[0].n_items;
    auto noutput_items = work_output[0].n_items;

	int i = 0;
	int o = 0;
	gr_complex symbols[48];
	gr_complex current_symbol[64];

	dout << "FRAME EQUALIZER: input " << ninput_items << "  output " << noutput_items << std::endl;



	while((i < ninput_items) && (o < noutput_items)) {

		// get_tags_in_window(tags, 0, i, i + 1, pmt::string_to_symbol("wifi_start"));
        tags = work_input[0].tags_in_window(i, i+1);

		// new frame
		if(tags.size()) {
            // ntags_rx++;
			d_current_symbol = 0;
			d_frame_symbols = 0;
			d_frame_mod = d_bpsk;

            auto tag_val = pmtf::get_scalar_value<double>(tags.front().value);
			
			// pmtf::pmt_scalar<double>::from_buffer(
            //                           tags.front().value->buffer_pointer()+4)
            //                           ->value();
			d_freq_offset_from_synclong = tag_val * d_bw / (2 * M_PI);
			d_epsilon0 = tag_val * d_bw / (2 * M_PI * d_freq);
			d_er = 0;

			dout << "epsilon: " << d_epsilon0 << std::endl;
            // std::cout << "rx: " << ntags_rx << std::endl;

			// std::cout << "tag on " << tags.front().offset << std::endl;;
		}

		// not interesting -> skip
		if(d_current_symbol > (d_frame_symbols + 2)) {
			i++;
			continue;
		}

		std::memcpy(current_symbol, in + i*64, 64*sizeof(gr_complex));

		// std::cout << "current_symbol = [";
		// for (int i=0; i<64; i++)
		// {
		// 	std::cout << real(current_symbol[i]) << "+1j*" << imag(current_symbol[i]) << ",";
		// }
		// std::cout << "];" << std::endl;

		// compensate sampling offset
		for(int i = 0; i < 64; i++) {
			current_symbol[i] *= exp(gr_complex(0, 2*M_PI*d_current_symbol*80*(d_epsilon0 + d_er)*(i-32)/64));
		}

		// std::cout << "current_symbol = [";
		// for (int i=0; i<64; i++)
		// {
		// 	std::cout << real(current_symbol[i]) << "+1j*" << imag(current_symbol[i]) << ",";
		// }
		// std::cout << "];" << std::endl;


		gr_complex p = equalizer::base::POLARITY[(d_current_symbol - 2) % 127];

		double beta;
		if(d_current_symbol < 2) {
			beta = arg(
					current_symbol[11] -
					current_symbol[25] +
					current_symbol[39] +
					current_symbol[53]);

		} else {
			beta = arg(
					(current_symbol[11] *  p) +
					(current_symbol[39] *  p) +
					(current_symbol[25] *  p) +
					(current_symbol[53] * -p));
		}

		double er = arg(
				(conj(d_prev_pilots[0]) * current_symbol[11] *  p) +
				(conj(d_prev_pilots[1]) * current_symbol[25] *  p) +
				(conj(d_prev_pilots[2]) * current_symbol[39] *  p) +
				(conj(d_prev_pilots[3]) * current_symbol[53] * -p));

		er *= d_bw / (2 * M_PI * d_freq * 80);

		if(d_current_symbol < 2) {
			d_prev_pilots[0] = current_symbol[11];
			d_prev_pilots[1] = -current_symbol[25];
			d_prev_pilots[2] = current_symbol[39];
			d_prev_pilots[3] = current_symbol[53];
		} else {
			d_prev_pilots[0] = current_symbol[11] *  p;
			d_prev_pilots[1] = current_symbol[25] *  p;
			d_prev_pilots[2] = current_symbol[39] *  p;
			d_prev_pilots[3] = current_symbol[53] * -p;
		}

		// compensate residual frequency offset
		for(int i = 0; i < 64; i++) {
			current_symbol[i] *= exp(gr_complex(0, -beta));
		}

		// std::cout << "current_symbol = [";
		// for (int i=0; i<64; i++)
		// {
		// 	std::cout << real(current_symbol[i]) << "+1j*" << imag(current_symbol[i]) << ",";
		// }
		// std::cout << "];" << std::endl;


		// update estimate of residual frequency offset
		if(d_current_symbol >= 2) {

			double alpha = 0.1;
			d_er = (1-alpha) * d_er + alpha * er;
		}

		// do equalization
		d_equalizer->equalize(current_symbol, d_current_symbol,
				symbols, out + o * 48, d_frame_mod);


		// // matlab print
		// std::cout << "current_symbol = [";
		// for (int i=0; i<64; i++)
		// {
		// 	std::cout << real(current_symbol[i]) << "+1j*" << imag(current_symbol[i]) << ",";
		// }
		// std::cout << "];" << std::endl;

		// signal field
		if(d_current_symbol == 2) {

			if(decode_signal_field(out + o * 48)) {
				// std::cout << "PACKET with " << d_frame_bytes << std::endl;
				
				auto d = pmtf::map<std::string>(
					{
						{"frame_bytes",d_frame_bytes},
						{"encoding",d_frame_bytes},
						{"snr",d_frame_bytes},
						{"freq",d_frame_bytes},
						{"freq_offset",d_frame_bytes},
						{"wifi_start",d_frame_bytes},
					}
				);
				// pmtf::wrap dict = pmt::make_dict();
				// dict = pmt::dict_add(dict, pmt::mp("frame_bytes"),
				// 					pmt::from_uint64(d_frame_bytes));
				// dict = pmt::dict_add(dict, pmt::mp("encoding"),
				// 					pmt::from_uint64(d_frame_encoding));
				// dict = pmt::dict_add(dict, pmt::mp("snr"),
				// 					pmt::from_double(d_equalizer->get_snr()));
				// dict = pmt::dict_add(dict, pmt::mp("freq"), pmt::from_double(d_freq));
				// dict = pmt::dict_add(dict, pmt::mp("freq_offset"),
				// 					pmt::from_double(d_freq_offset_from_synclong));
				// work_output[0].add_tag(work_output[0].nitems_written() + o,
				// 			pmt::string_to_symbol("wifi_start"), dict,
				// 			pmt::string_to_symbol(alias()));

			}
		}

		if(d_current_symbol > 2) {
			o++;
			// pmtf::wrap pdu = pmt::make_dict();
			// message_port_pub(pmt::mp("symbols"), pmt::cons(pmt::make_dict(), pmt::init_c32vector(48, symbols)));
		}

		i++;
		d_current_symbol++;
	}



            // FILE *pFile;
            // pFile = fopen("/tmp/newsched_feq_in.fc32", "wb");
            // fwrite(in, 64*sizeof(gr_complex), i , pFile);
			// fclose(pFile);
    work_input[0].n_consumed = i;
    work_output[0].n_produced = o;
	// consume(0, i);
	return work_return_code_t::WORK_OK;
}

bool
frame_equalizer_cpu::decode_signal_field(uint8_t *rx_bits) {

	static ofdm_param ofdm(BPSK_1_2);
	static frame_param frame(ofdm, 0);

	deinterleave(rx_bits);
	uint8_t *decoded_bits = d_decoder.decode(&ofdm, &frame, d_deinterleaved);

	return parse_signal(decoded_bits);
}

void
frame_equalizer_cpu::deinterleave(uint8_t *rx_bits) {
	for(int i = 0; i < 48; i++) {
		d_deinterleaved[i] = rx_bits[interleaver_pattern[i]];
	}
}

bool
frame_equalizer_cpu::parse_signal(uint8_t *decoded_bits) {

	int r = 0;
	d_frame_bytes = 0;
	bool parity = false;
	for(int i = 0; i < 17; i++) {
		parity ^= decoded_bits[i];

		if((i < 4) && decoded_bits[i]) {
			r = r | (1 << i);
		}

		if(decoded_bits[i] && (i > 4) && (i < 17)) {
			d_frame_bytes = d_frame_bytes | (1 << (i-5));
		}
	}

	if(parity != decoded_bits[17]) {
		dout << "SIGNAL: wrong parity" << std::endl;
		return false;
	}

	switch(r) {
	case 11:
		d_frame_encoding = 0;
		d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 24);
		d_frame_mod = d_bpsk;
		dout << "Encoding: 3 Mbit/s   ";
		break;
	case 15:
		d_frame_encoding = 1;
		d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 36);
		d_frame_mod = d_bpsk;
		dout << "Encoding: 4.5 Mbit/s   ";
		break;
	case 10:
		d_frame_encoding = 2;
		d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 48);
		d_frame_mod = d_qpsk;
		dout << "Encoding: 6 Mbit/s   ";
		break;
	case 14:
		d_frame_encoding = 3;
		d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 72);
		d_frame_mod = d_qpsk;
		dout << "Encoding: 9 Mbit/s   ";
		break;
	case 9:
		d_frame_encoding = 4;
		d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 96);
		d_frame_mod = d_16qam;
		dout << "Encoding: 12 Mbit/s   ";
		break;
	case 13:
		d_frame_encoding = 5;
		d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 144);
		d_frame_mod = d_16qam;
		dout << "Encoding: 18 Mbit/s   ";
		break;
	case 8:
		d_frame_encoding = 6;
		d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 192);
		d_frame_mod = d_64qam;
		dout << "Encoding: 24 Mbit/s   ";
		break;
	case 12:
		d_frame_encoding = 7;
		d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 216);
		d_frame_mod = d_64qam;
		dout << "Encoding: 27 Mbit/s   ";
		break;
	default:
		dout << "unknown encoding" << std::endl;
		return false;
	}

	// mylog(boost::format("encoding: %1% - length: %2% - symbols: %3%")
			// % d_frame_encoding % d_frame_bytes % d_frame_symbols);
	return true;
}

const int
frame_equalizer_cpu::interleaver_pattern[48] = {
	 0, 3, 6, 9,12,15,18,21,
	24,27,30,33,36,39,42,45,
	 1, 4, 7,10,13,16,19,22,
	25,28,31,34,37,40,43,46,
	 2, 5, 8,11,14,17,20,23,
	26,29,32,35,38,41,44,47};

} // namespace wifi
} // namespace gr