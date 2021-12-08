#pragma once

#include <gnuradio/wifi/sync_long.hh>
#include <list>
#include <tuple>
#include <gnuradio/filter/fir_filter.hh>
#include <gnuradio/fft/fftw_fft.hh>
using namespace std;

namespace gr {
namespace wifi {

class sync_long_cpu : public sync_long
{
public:
    sync_long_cpu(const block_args& args);
    virtual work_return_code_t work(std::vector<block_work_input_sptr>& work_input,
                                    std::vector<block_work_output_sptr>& work_output) override;

static bool compare_abs(const std::pair<gr_complex, int>& first, const std::pair<gr_complex, int>& second) {
	return abs(get<0>(first)) > abs(get<0>(second));
}

void search_frame_start() {

	// sort list (highest correlation first)
	assert(d_cor.size() == SYNC_LENGTH);
	d_cor.sort(compare_abs);

	// copy list in vector for nicer access
	vector<pair<gr_complex, int> > vec(d_cor.begin(), d_cor.end());
	d_cor.clear();

	// in case we don't find anything use SYNC_LENGTH
	d_frame_start = SYNC_LENGTH;

	for(int i = 0; i < 3; i++) {
		for(int k = i + 1; k < 4; k++) {
			gr_complex first;
			gr_complex second;
			if(get<1>(vec[i]) > get<1>(vec[k])) {
				first = get<0>(vec[k]);
				second = get<0>(vec[i]);
			} else {
				first = get<0>(vec[i]);
				second = get<0>(vec[k]);
			}
			int diff  = abs(get<1>(vec[i]) - get<1>(vec[k]));
			if(diff == 64) {
				d_frame_start = min(get<1>(vec[i]), get<1>(vec[k]));
				d_freq_offset = arg(first * conj(second)) / 64;
				// nice match found, return immediately
				return;

			} else if(diff == 63) {
				d_frame_start = min(get<1>(vec[i]), get<1>(vec[k]));
				d_freq_offset = arg(first * conj(second)) / 63;
			} else if(diff == 65) {
				d_frame_start = min(get<1>(vec[i]), get<1>(vec[k]));
				d_freq_offset = arg(first * conj(second)) / 65;
			}
		}
	}
}

private:
	const bool d_log;
	const bool d_debug;
	
	int         d_count;
	int         d_offset = 0;
	enum {SYNC, COPY, RESET} d_state = SYNC;
	const size_t  SYNC_LENGTH;
	int         d_frame_start;
	float       d_freq_offset;
	double      d_freq_offset_short;

	gr_complex *d_correlation;
	list<pair<gr_complex, int> > d_cor;
	std::vector<gr::tag_t> d_tags;
	gr::filter::kernel::fir_filter_ccc d_fir;

	static const std::vector<gr_complex> LONG;

	size_t ntags_tx = 0;
};

} // namespace wifi
} // namespace gr