#pragma once

#include <gnuradio/wifi/sync_short.hh>

#include <pmtf/wrap.hpp>
#include <pmtf/string.hpp>
#include <pmtf/scalar.hpp>

namespace gr {
namespace wifi {

class sync_short_cpu : public sync_short
{
public:
    sync_short_cpu(const block_args& args);
    virtual work_return_code_t work(std::vector<block_work_input_sptr>& work_input,
                                    std::vector<block_work_output_sptr>& work_output) override;

void insert_tag(uint64_t item, double freq_offset, uint64_t input_item, block_work_output_sptr& work_output) {
	// mylog(boost::format("frame start at in: %2% out: %1%") % item % input_item);

	const auto key = pmtf::string("wifi_start");
	const auto value = pmtf::scalar<double>(freq_offset);
	const auto srcid = pmtf::string(name());
	work_output->add_tag(item, key, value, srcid);

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