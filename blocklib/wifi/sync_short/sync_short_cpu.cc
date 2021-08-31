#include "sync_short_cpu.hh"

static const int MIN_GAP = 480;
static const int MAX_SAMPLES = 540 * 80;

namespace gr {
namespace wifi {

sync_short::sptr sync_short::make_cpu(const block_args& args)
{
    return std::make_shared<sync_short_cpu>(args);
}

sync_short_cpu::sync_short_cpu(const sync_short::block_args& args)
    : sync_short(args),
      d_log(args.log),
      d_debug(args.debug),
      MIN_PLATEAU(args.min_plateau),
      d_threshold(args.threshold)
{

    set_tag_propagation_policy(tag_propagation_policy_t::TPP_DONT);
}

work_return_code_t sync_short_cpu::work(std::vector<block_work_input>& work_input,
                                             std::vector<block_work_output>& work_output)
{
    const gr_complex* in = (const gr_complex*)work_input[0].items();
    const gr_complex* in_abs = (const gr_complex*)work_input[1].items();
    const float* in_cor = (const float*)work_input[2].items();
    gr_complex* out = (gr_complex*)work_output[0].items();

    int noutput = work_output[0].n_items;
    int ninput = std::min(std::min(work_input[0].n_items, work_input[1].n_items),
                          work_input[2].n_items);

    // std::cout << "SHORT noutput : " << noutput << " ninput: " << work_input[0].n_items
            //   << std::endl;

    // int o = std::min(ninput, noutput);
    // consume_each(o, work_input);
    // produce_each(o, work_output);
    // return work_return_code_t::WORK_OK;

    switch (d_state) {

    case SEARCH: {
        int i;

        for (i = 0; i < ninput; i++) {
            // std::cout << work_input[0].nitems_read()+i << " " << work_input[1].nitems_read()+i << " " << work_input[2].nitems_read()+i << std::endl;
            if (in_cor[i] > d_threshold) {
                if (d_plateau < MIN_PLATEAU) {
                    d_plateau++;

                } else {
                    d_state = COPY;
                    d_copied = 0;
                    d_freq_offset = arg(in_abs[i]) / 16;
                    d_plateau = 0;
                    insert_tag(work_output[0].nitems_written(),
                               d_freq_offset,
                               work_input[0].nitems_read() + i,
                               work_output[0]);
                    GR_LOG_DEBUG(_debug_logger, "SHORT Frame!")
                    break;
                }
            } else {
                d_plateau = 0;
            }
        }

        // std::cout << work_input[0].buffer->total_read() << " " << 
        //              work_input[1].buffer->total_read() << " " << 
        //              work_input[2].buffer->total_read() << " " << 
        //              work_output[0].buffer->total_written() << " " << 
        //              i << " " << 0 << " " << std::endl;

        consume_each(i, work_input);
        produce_each(0, work_output);
        return work_return_code_t::WORK_OK;
    }

    case COPY: {

        // std::cout << work_input[0].nitems_read() << " " << work_input[1].nitems_read() << " " << work_input[2].nitems_read() << std::endl;
        // throw std::runtime_error("nothing");

        int o = 0;
        while (o < ninput && o < noutput && d_copied < MAX_SAMPLES) {
            // std::cout << work_input[0].nitems_read()+o << " " << work_input[1].nitems_read()+o << " " << work_input[2].nitems_read()+o << std::endl;
            if (in_cor[o] > d_threshold) {
                if (d_plateau < MIN_PLATEAU) {
                    d_plateau++;

                    // there's another frame
                } else if (d_copied > MIN_GAP) {
                    d_copied = 0;
                    d_plateau = 0;
                    d_freq_offset = arg(in_abs[o]) / 16;
                    insert_tag(work_output[0].nitems_written() + o,
                               d_freq_offset,
                               work_input[0].nitems_read() + o,
                               work_output[0]);
                    GR_LOG_DEBUG(_debug_logger, "SHORT Frame!");
                    break;
                }

            } else {
                d_plateau = 0;
            }

            out[o] = in[o] * exp(gr_complex(0, -d_freq_offset * d_copied));
            o++;
            d_copied++;
        }

        if (d_copied == MAX_SAMPLES) {
            d_state = SEARCH;
        }

        GR_LOG_DEBUG(_debug_logger, "SHORT copied {}", o);

        // std::cout << work_input[0].buffer->total_read() << " " << 
        //              work_input[1].buffer->total_read() << " " << 
        //              work_input[2].buffer->total_read() << " " << 
        //              work_output[0].buffer->total_written() << " " << 
        //              o << " " << o << " " << std::endl;

        consume_each(o, work_input);
        produce_each(o, work_output);
        return work_return_code_t::WORK_OK;
    }
    }

    throw std::runtime_error("sync short: unknown state");
}


} // namespace wifi
} // namespace gr