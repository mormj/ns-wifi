#pragma once

#include <gnuradio/wifi/sync_short.hh>

#include <pmt/pmt.h>

namespace gr {
namespace wifi {

class sync_short_cpu : public sync_short
{
public:
    sync_short_cpu(const block_args& args);
    virtual work_return_code_t work(std::vector<block_work_input>& work_input,
                                    std::vector<block_work_output>& work_output) override;

void insert_tag(uint64_t item, double freq_offset, uint64_t input_item, block_work_output& work_output) {
	// mylog(boost::format("frame start at in: %2% out: %1%") % item % input_item);

	const pmt::pmt_t key = pmt::string_to_symbol("wifi_start");
	const pmt::pmt_t value = pmt::from_double(freq_offset);
	const pmt::pmt_t srcid = pmt::string_to_symbol(name());
	work_output.add_tag(item, key, value, srcid);

}

private:
	enum {SEARCH, COPY} d_state = SEARCH;
	int d_copied = 0;
	int d_plateau = 0;
	float d_freq_offset = 0;
	const bool d_log;
	const bool d_debug;
	const unsigned int MIN_PLATEAU;
	const double d_threshold;
};

} // namespace wifi
} // namespace gr