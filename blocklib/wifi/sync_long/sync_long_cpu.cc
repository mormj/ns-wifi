#include "sync_long_cpu.hh"

#include <pmtf/wrap.hpp>
#include <pmtf/scalar.hpp>
#include <pmtf/string.hpp>

static const int MIN_GAP = 480;
static const int MAX_SAMPLES = 540 * 80;

namespace gr {
namespace wifi {

sync_long::sptr sync_long::make_cpu(const block_args& args)
{
    return std::make_shared<sync_long_cpu>(args);
}

sync_long_cpu::sync_long_cpu(const sync_long::block_args& args)
    : block("sync_long_cpu"), sync_long(args),
      d_log(args.log),
      d_debug(args.debug),
      SYNC_LENGTH(args.sync_length),
      d_fir(gr::filter::kernel::fir_filter_ccc(LONG))
{
    d_correlation = gr::fft::malloc_complex(8192);
    set_tag_propagation_policy(tag_propagation_policy_t::TPP_DONT);
}

work_return_code_t sync_long_cpu::work(std::vector<block_work_input>& work_input,
                                            std::vector<block_work_output>& work_output)
{
    auto in = work_input[0].items<gr_complex>();
    auto in_delayed = work_input[1].items<gr_complex>();
    auto out = work_output[0].items<gr_complex>();
    auto noutput = work_output[0].n_items;

    GR_LOG_DEBUG(_debug_logger,
                 "LONG ninput[0] {}  ninput[1] {} noutput {} state {}",
                 work_input[0].n_items,
                 work_input[1].n_items,
                 noutput,
                 d_state);

    int ninput = std::min(std::min(work_input[0].n_items, work_input[1].n_items), 8192);

    const uint64_t nread = work_input[0].nitems_read();
    // get_tags_in_range(d_tags, 0, nread, nread + ninput);
    d_tags = work_input[0].tags_in_window(0, ninput);
    if (d_tags.size()) {
        std::sort(d_tags.begin(), d_tags.end(), gr::tag_t::offset_compare);

        const uint64_t offset = d_tags.front().offset;

        if (offset > nread) {
            ninput = offset - nread;
        } else {
            if (d_offset && (d_state == SYNC)) {
                throw std::runtime_error("wtf");
            }
            if (d_state == COPY) {
                d_state = RESET;
            }
            d_freq_offset_short = pmtf::get_scalar_value<double>(d_tags.front().value);
        }
    }


    int i = 0;
    int o = 0;

    switch (d_state) {

    case SYNC:
        d_fir.filterN(d_correlation, in, std::min((int)SYNC_LENGTH, std::max(ninput - 63, 0)));

        while (i + 63 < ninput) {

            d_cor.push_back(pair<gr_complex, int>(d_correlation[i], d_offset));

            i++;
            d_offset++;

            if (d_offset == (int)SYNC_LENGTH) {
                search_frame_start();
                // mylog(boost::format("LONG: frame start at %1%") % d_frame_start);
                GR_LOG_INFO(_debug_logger, "LONG: frame start at {}", d_frame_start);
                d_offset = 0;
                d_count = 0;
                d_state = COPY;

                break;
            }
        }

        break;

    case COPY:
        while (i < ninput && o < noutput) {

            int rel = d_offset - d_frame_start;

            if (!rel) {
                // add_item_tag(0, nitems_written(0),
                // 		pmt::string_to_symbol("wifi_start"),
                // 		pmt::from_double(d_freq_offset_short - d_freq_offset),
                // 		pmt::string_to_symbol(name()));
                work_output[0].add_tag(
                    work_output[0].nitems_written(),
                    pmtf::string("wifi_start"),
                    pmtf::scalar<double>(d_freq_offset_short - d_freq_offset),
                    pmtf::string(name()));

                // std::cout << "             sl tx: " << ++ntags_tx << " ? " <<
                // work_output[0].nitems_written() << std::endl;
            }

            if (rel >= 0 && (rel < 128 || ((rel - 128) % 80) > 15)) {
                out[o] = in_delayed[i] * exp(gr_complex(0, d_offset * d_freq_offset));
                o++;
            }

            i++;
            d_offset++;
        }

        break;

    case RESET: {
        while (o < noutput) {
            if (((d_count + o) % 64) == 0) {
                d_offset = 0;
                d_state = SYNC;
                break;
            } else {
                out[o] = 0;
                o++;
            }
        }

        break;
    }
    }

    // dout << "produced : " << o << " consumed: " << i << std::endl;
    GR_LOG_DEBUG(_debug_logger, "produced: {}  consumed: {}", o, i);
    d_count += o;

    consume_each(i, work_input);
    produce_each(o, work_output);
    return work_return_code_t::WORK_OK;
}

const std::vector<gr_complex> sync_long_cpu::LONG = {

    gr_complex(-0.0455, -1.0679), gr_complex(0.3528, -0.9865),
    gr_complex(0.8594, 0.7348),   gr_complex(0.1874, 0.2475),
    gr_complex(0.5309, -0.7784),  gr_complex(-1.0218, -0.4897),
    gr_complex(-0.3401, -0.9423), gr_complex(0.8657, -0.2298),
    gr_complex(0.4734, 0.0362),   gr_complex(0.0088, -1.0207),
    gr_complex(-1.2142, -0.4205), gr_complex(0.2172, -0.5195),
    gr_complex(0.5207, -0.1326),  gr_complex(-0.1995, 1.4259),
    gr_complex(1.0583, -0.0363),  gr_complex(0.5547, -0.5547),
    gr_complex(0.3277, 0.8728),   gr_complex(-0.5077, 0.3488),
    gr_complex(-1.1650, 0.5789),  gr_complex(0.7297, 0.8197),
    gr_complex(0.6173, 0.1253),   gr_complex(-0.5353, 0.7214),
    gr_complex(-0.5011, -0.1935), gr_complex(-0.3110, -1.3392),
    gr_complex(-1.0818, -0.1470), gr_complex(-1.1300, -0.1820),
    gr_complex(0.6663, -0.6571),  gr_complex(-0.0249, 0.4773),
    gr_complex(-0.8155, 1.0218),  gr_complex(0.8140, 0.9396),
    gr_complex(0.1090, 0.8662),   gr_complex(-1.3868, -0.0000),
    gr_complex(0.1090, -0.8662),  gr_complex(0.8140, -0.9396),
    gr_complex(-0.8155, -1.0218), gr_complex(-0.0249, -0.4773),
    gr_complex(0.6663, 0.6571),   gr_complex(-1.1300, 0.1820),
    gr_complex(-1.0818, 0.1470),  gr_complex(-0.3110, 1.3392),
    gr_complex(-0.5011, 0.1935),  gr_complex(-0.5353, -0.7214),
    gr_complex(0.6173, -0.1253),  gr_complex(0.7297, -0.8197),
    gr_complex(-1.1650, -0.5789), gr_complex(-0.5077, -0.3488),
    gr_complex(0.3277, -0.8728),  gr_complex(0.5547, 0.5547),
    gr_complex(1.0583, 0.0363),   gr_complex(-0.1995, -1.4259),
    gr_complex(0.5207, 0.1326),   gr_complex(0.2172, 0.5195),
    gr_complex(-1.2142, 0.4205),  gr_complex(0.0088, 1.0207),
    gr_complex(0.4734, -0.0362),  gr_complex(0.8657, 0.2298),
    gr_complex(-0.3401, 0.9423),  gr_complex(-1.0218, 0.4897),
    gr_complex(0.5309, 0.7784),   gr_complex(0.1874, -0.2475),
    gr_complex(0.8594, -0.7348),  gr_complex(0.3528, 0.9865),
    gr_complex(-0.0455, 1.0679),  gr_complex(1.3868, -0.0000),

};

} // namespace wifi
} // namespace gr