#include "sync_long_cuda.hh"

#include <pmt/pmt.h>

#include <gnuradio/helper_cuda.h>


static const int MIN_GAP = 480;
static const int MAX_SAMPLES = 540 * 80;

extern void exec_multiply_kernel_ccc(cuFloatComplex* in1,
                                     cuFloatComplex* in2,
                                     cuFloatComplex* out,
                                     int n,
                                     int grid_size,
                                     int block_size,
                                     cudaStream_t stream);

extern void exec_remove_cp(cuFloatComplex* in,
                           cuFloatComplex* out,
                           int symlen,
                           int cplen,
                           int n,
                           int grid_size,
                           int block_size,
                           cudaStream_t stream);
                           

extern void exec_remove_cp_freqcorr(cuFloatComplex* in,
    cuFloatComplex* out,
    int symlen,
    int cplen,
    int n,
    int grid_size,
    int block_size,
    float freqoff,
    int start_sym,
    cudaStream_t stream);

extern void exec_freq_correction(cuFloatComplex* in,
                                 cuFloatComplex* out,
                                 float freq_offset,
                                 float start_idx,
                                 int n,
                                 int grid_size,
                                 int block_size,
                                 cudaStream_t stream);

namespace gr {
namespace wifi {

sync_long::sptr sync_long::make_cuda(const block_args& args)
{
    return std::make_shared<sync_long_cuda>(args);
}

sync_long_cuda::sync_long_cuda(const sync_long::block_args& args)
    : block("sync_long_cuda"), sync_long(args), d_log(args.log), d_debug(args.debug)
{
    cudaStreamCreate(&d_stream);

    checkCudaErrors(cufftCreate(&d_plan));
    checkCudaErrors(cufftSetStream(d_plan, d_stream));

    size_t workSize;
    checkCudaErrors(cufftMakePlanMany(
        d_plan, 1, &d_fftsize, NULL, 1, 1, NULL, 1, 1, CUFFT_C2C, 1, &workSize));

    checkCudaErrors(
        cudaMalloc((void**)&d_dev_training_freq, sizeof(cufftComplex) * d_fftsize * 1));
    checkCudaErrors(cudaMalloc((void**)&d_dev_in, sizeof(cufftComplex) * d_fftsize * 1));

    checkCudaErrors(cudaMemset(d_dev_training_freq, 0, d_fftsize * sizeof(cufftComplex)));

    checkCudaErrors(cudaMemcpy(d_dev_training_freq,
                               &LONG[0],
                               64 * sizeof(cufftComplex),
                               cudaMemcpyHostToDevice));
    // checkCudaErrors(cudaMemcpy(d_dev_training_freq + 64,
    //                            &LONG[0],
    //                            64 * sizeof(cufftComplex),
    //                            cudaMemcpyHostToDevice));

    checkCudaErrors(
        cufftExecC2C(d_plan, d_dev_training_freq, d_dev_training_freq, CUFFT_FORWARD));
    cudaStreamSynchronize(d_stream);

    // get_block_and_grid_multiply(&d_min_grid_size, &d_block_size);
    d_block_size = 1024;

    set_tag_propagation_policy(tag_propagation_policy_t::TPP_DONT);

    // set_output_multiple(64); // ??
}

work_return_code_t sync_long_cuda::work(std::vector<block_work_input>& work_input,
                                        std::vector<block_work_output>& work_output)
{
    // Since this is a decimating block, forecast for noutput
    // int ninput = std::min(work_input[0].n_items, work_input[1].n_items);
    auto ninput = work_input[0].n_items;
    auto noutput = work_output[0].n_items * 64;
    // if (noutput * 64 > ninput * 80 / 64 )
    // {
    //     return work_return_code_t::WORK_INSUFFICIENT_INPUT_ITEMS;
    // }

    auto in = static_cast<const gr_complex*>(work_input[0].items());
    auto out = static_cast<gr_complex*>(work_output[0].items());


    GR_LOG_DEBUG(_debug_logger,
                 "LONG ninput[0] {}  ninput[1] {} noutput {} state {}",
                 work_input[0].n_items,
                 work_input[1].n_items,
                 noutput,
                 d_state);


    auto nread = work_input[0].nitems_read();
    auto nwritten = work_output[0].nitems_written();
    tags = work_input[0].tags_in_window(0, ninput);

    // std::cout << tags.size() << std::endl;

    int nconsumed = 0;
    int nproduced = 0;

    size_t tag_idx = 0;
    while (true) {
        auto tag = &tags[tag_idx];
        tag_t* next_tag = nullptr;
        if (tag_idx < tags.size() - 1) {
            next_tag = &tags[tag_idx + 1];
        }

        if (d_state == FINISH_LAST_FRAME) {
            auto max_consume = ninput - nconsumed;
            auto max_produce = noutput - nproduced;
            if (tag_idx < tags.size()) {
                // only consume up to the next tag
                max_consume = tags[tag_idx].offset - (nread + nconsumed);
                if (max_consume < 80 ||
                    max_produce < 64) { // need an entire OFDM symbol to do anything here
                    nconsumed = tags[tag_idx].offset - nread;
                    d_state = WAITING_FOR_TAG;
                    continue;
                }
            } else { // no more tags
                if (max_consume < 80 ||
                    max_produce < 64) { // need an entire OFDM symbol to do anything here
                    // nconsumed += max_consume;
                    break;
                }
            }

            auto nsyms = std::min(max_consume / 80, max_produce / 64);
            auto gridSize = (80 * nsyms + d_block_size - 1) / d_block_size;
            // exec_remove_cp((cuFloatComplex*)in + nconsumed,
            //                (cuFloatComplex*)out + nproduced,
            //                80,
            //                16,
            //                80 * nsyms,
            //                gridSize,
            //                d_block_size,
            //                d_stream);
            // // cudaStreamSynchronize(d_stream);


            exec_remove_cp_freqcorr((cuFloatComplex*)in + nconsumed,
                           (cuFloatComplex*)out + nproduced,
                           80,
                           16,
                           80 * nsyms,
                           gridSize,
                           d_block_size,
                           -d_freq_offset,
                           d_offset,
                           d_stream);
            // cudaStreamSynchronize(d_stream);

            int i = 80 * nsyms;
            int o = 64 * nsyms;
            nconsumed += i;
            nproduced += o;

            d_offset += i;

        } else { // WAITING_FOR_TAG

            if (tag_idx < tags.size()) {
                auto offset = tags[tag_idx].offset;

                d_freq_offset_short = pmt::to_double(tags[0].value);

                if (offset - nread + d_fftsize <= ninput &&
                    (noutput - nproduced) >= 128) {

                    checkCudaErrors(cufftExecC2C(d_plan,
                                                 (cufftComplex*)&in[offset - nread],
                                                 d_dev_in,
                                                 CUFFT_FORWARD));


                    auto gridSize = (d_fftsize + d_block_size - 1) / d_block_size;
                    exec_multiply_kernel_ccc(d_dev_in,
                                             d_dev_training_freq,
                                             d_dev_in,
                                             d_fftsize,
                                             gridSize,
                                             d_block_size,
                                             d_stream);

                    checkCudaErrors(
                        cufftExecC2C(d_plan, d_dev_in, d_dev_in, CUFFT_INVERSE));

                    // Find the peak
                    std::vector<gr_complex> host_data(d_fftsize);
                    checkCudaErrors(cudaMemcpyAsync(host_data.data(),
                                                    d_dev_in,
                                                    d_fftsize * sizeof(cufftComplex),
                                                    cudaMemcpyDeviceToHost,
                                                    d_stream));

                    cudaStreamSynchronize(d_stream);
                    std::vector<float> abs_corr(d_fftsize);
                    // std::cout << "freq_corr = [";

                    size_t max_index = 0;
                    size_t max_index_2 = 0;
                    float max_value = 0.0;
                    float max_value_2 = 0.0;
                    gr_complex corr_1;
                    gr_complex corr_2;

                    // for (size_t i = 0; i < host_data.size(); i++) {
                    //     abs_corr[i] = std::abs(host_data[i]);
                    //     // std::cout << abs_corr[i];
                    //     // if (i < host_data.size()-1)
                    //     // std::cout << ",";

                    //     if (abs_corr[i] > max_value) {
                    //         max_value_2 = max_value;
                    //         max_index_2 = max_index;
                    //         max_value = abs_corr[i];
                    //         max_index = i;
                    //     }
                    //     else if (abs_corr[i] > max_value_2)
                    //     {
                    //         max_value_2 = abs_corr[i];
                    //         max_index_2 = i;
                    //     }
                    // }
                    // std::cout << "abs_corr = [";
                    for (size_t i = 0; i < host_data.size(); i++) {
                        abs_corr[i] = std::abs(host_data[i]);
                        // std::cout << abs_corr[i];
                        // if (i < host_data.size() - 1)
                        //     std::cout << ",";

                        if (abs_corr[i] > max_value) {
                            max_value = abs_corr[i];
                            max_index = i;
                        }
                    }
                    // std::cout << "];" << std::endl;
                    for (size_t i = 0; i < host_data.size(); i++) {
                        if (abs_corr[i] > max_value_2 && i != max_index) {
                            max_value_2 = abs_corr[i];
                            max_index_2 = i;
                        }
                    }

                    bool valid = false;
                    if (max_index_2 > max_index) {
                        auto diff = max_index_2 - max_index;
                        if (diff <= 65 && diff >= 63) {

                            d_freq_offset =
                                arg(host_data[max_index] * conj(host_data[max_index_2])) /
                                (max_index_2 - max_index) ;
                            max_index = max_index_2;
                            valid = true;
                        }
                    } else {
                        auto diff = max_index - max_index_2;
                        if (diff <= 65 && diff >= 63) {
                            d_freq_offset =
                                arg(host_data[max_index_2] * conj(host_data[max_index])) /
                                (max_index - max_index_2) ;
                            
                            valid = true;
                        }
                    }

                    if (valid)
                    {
                            // d_freq_offset = -.00248403;
                    // size_t max_index = 297;

                    // Copy the LTF symbols
                    // offset and nread should always be equal
                    size_t copy_index = 0;
                    if (max_index > (160 - 32 - 1)) {
                        copy_index = max_index - 160 + 32 + 1;
                    }
                    // if (max_index > (160 - 1)) {
                    //     copy_index = max_index - 160 + 1;
                    // }
                    


                    // checkCudaErrors(cudaMemcpyAsync(out + nproduced,
                    //                                 in + (offset - nread + copy_index),
                    //                                 sizeof(gr_complex) * 128,
                    //                                 cudaMemcpyDeviceToDevice,
                    //                                 d_stream));

                    exec_freq_correction((cuFloatComplex*)in + (offset - nread + copy_index),
                                                    (cuFloatComplex*)out + nproduced,
                                                    -d_freq_offset, // it gets negated in the kernel
                                                    0,
                                                    128,
                                                    1,
                                                    128,
                                                    d_stream);
                    cudaStreamSynchronize(d_stream);
                gr_complex host_in[128];
                gr_complex host_out[128];
                cudaMemcpy(host_in, in + (offset - nread + copy_index), 128*sizeof(gr_complex), cudaMemcpyDeviceToHost);
                cudaMemcpy(host_out, out + nproduced, 128*sizeof(gr_complex), cudaMemcpyDeviceToHost);
                // FILE *pFile;
                // pFile = fopen("/tmp/sync_long_freqcorrect.m", "w");
                // fprintf(pFile, "x = [");
                // for (int i=0; i<128; i++)
                // {
                //     fprintf(pFile, "%.6f+%.6fj,", real(host_out[i]), imag(host_out[i]));
                // }
                // fprintf(pFile, "];\n");
                // fprintf(pFile, "y = [");
                // for (int i=0; i<128; i++)
                // {
                //     fprintf(pFile, "%.6f+%.6fj,", real(host_in[i]), imag(host_in[i]));
                // }
                // fprintf(pFile, "];\n");
                // // fwrite(rx_bits, 1, frame_info.n_sym * 48 , pFile);
                // fclose(pFile);

                    d_offset = 160;

                    // checkCudaErrors(cudaMemcpyAsync(tmp,
                    //                                 in + (offset - nread + copy_index),
                    //                                 sizeof(gr_complex) * 128,
                    //                                 cudaMemcpyDeviceToHost,
                    //                                 d_stream));
                   


                    const pmt::pmt_t key = pmt::string_to_symbol("wifi_start");
                    const pmt::pmt_t value = // pmt::from_long(max_index);
                        pmt::from_double(d_freq_offset_short - d_freq_offset);
                    const pmt::pmt_t srcid = pmt::string_to_symbol(name());
                    work_output[0].add_tag(nwritten + nproduced / 64, key, value, srcid);
                    // std::cout << "SYNC LONG tag at " << nwritten + nproduced / 64 <<
                    // std::endl;
                    //   add_item_tag(0, nwritten + nproduced, key, value, srcid);
                    packet_cnt++;
                    if (packet_cnt % 1000 == 0) {
                        std::cout << "sync_long: " << packet_cnt << std::endl;
                    }

                    d_num_syms = 0;
                    d_state = FINISH_LAST_FRAME;
                    nconsumed = (offset - nread + copy_index + 128);
                    nproduced += 128;

                    }
                    tag_idx++;

                } else {
                    // not enough left with this tag
                    // clear up to the current tag
                    nconsumed = offset - nread;
                    break;
                }

            } else // out of tags
            {
                nconsumed = ninput;
                break;
            }
        }
    }
    cudaStreamSynchronize(d_stream);
    consume_each(nconsumed, work_input);
    assert(nproduced % 64 == 0);
    // std::cout << " nproduced " <<  nproduced / 64;
    produce_each(nproduced / 64, work_output);
    return work_return_code_t::WORK_OK;
}

const std::vector<gr_complex> sync_long_cuda::LONG = {

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