#!/usr/bin/env python3

from newsched import gr, blocks, streamops, fileio, math, wifi, filter, fft
from newsched.schedulers import mt, threadpool
import numpy as np
import argparse
import time
import signal, sys

def argParse():
    """Parses commandline args."""
    desc='Scrape the doxygen generated xml for docstrings to insert into python bindings'
    parser = argparse.ArgumentParser(description=desc)
    
    parser.add_argument("filename")
    parser.add_argument("--nthreads", type=int, default=1)

    return parser.parse_args()

def main():
    args = argParse()


    ##################################################
    # Variables
    ##################################################
    window_size = 48
    sync_length = 320
    samp_rate = 20000000
    lo_offset = 0
    freq = 2412000000
    chan_est = 0


    fg = gr.flowgraph()

 

    if 1: # through sync_short
        src = fileio.file_source(gr.sizeof_gr_complex, args.filename, False)
        pre_sync = wifi.pre_sync(48,1024*1024*2, impl=wifi.pre_sync.cuda)
        sync_short = wifi.sync_short(0.56, 2, True, False, impl=wifi.sync_short.cuda)
        sync_long = wifi.sync_long(sync_length, True, False, impl=wifi.sync_long.cuda)
        fft_blk = fft.fft_cc_fwd(64, np.ones(64), True, impl=fft.fft_cc_fwd.cuda)
        # fft_blk = fft.fft_cc_fwd(64, np.ones(64), True, impl=fft.fft_cc_fwd.cpu)
        packetize = wifi.packetize_frame(0,2412e6,20e6,False, False, impl=wifi.packetize_frame.cpu)
        decode = wifi.decode_packetized(False, False)

        buf_size = 2048*1024
        # fg.connect(src,0,hd,0)
        fg.connect(src, 0, pre_sync, 0).set_custom_buffer(gr.buffer_cuda_properties.make(gr.buffer_cuda_type.H2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 0, sync_short, 0).set_custom_buffer(gr.buffer_cuda_properties.make(gr.buffer_cuda_type.D2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 1, sync_short, 1).set_custom_buffer(gr.buffer_cuda_properties.make(gr.buffer_cuda_type.D2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 2, sync_short, 2).set_custom_buffer(gr.buffer_cuda_properties.make(gr.buffer_cuda_type.D2D).set_buffer_size(buf_size))
        # fg.connect(sync_short, 0, snk, 0).set_custom_buffer(gr.buffer_cuda_properties.make(gr.buffer_cuda_type.D2H).set_buffer_size(buf_size))
        # fg.connect(sync_long, 0, snk, 0)
        fg.connect(sync_short, 0, sync_long, 0).set_custom_buffer(gr.buffer_cuda_properties.make(gr.buffer_cuda_type.D2D).set_buffer_size(buf_size))
        fg.connect(sync_long, 0, fft_blk, 0).set_custom_buffer(gr.buffer_cuda_properties.make(gr.buffer_cuda_type.D2D).set_buffer_size(buf_size))
        # fg.connect(sync_long, 0, fft_blk, 0).set_custom_buffer(gr.buffer_cuda_properties.make(gr.buffer_cuda_type.D2H).set_buffer_size(buf_size))
        fg.connect(fft_blk, 0, packetize, 0).set_custom_buffer(gr.buffer_cuda_properties.make(gr.buffer_cuda_type.D2H).set_buffer_size(buf_size))
        # fg.connect(fft_blk, 0, packetize, 0).set_custom_buffer(gr.buffer_cpu_vmcirc_properties.make(gr.buffer_cpu_vmcirc_type.AUTO).set_buffer_size(buf_size))
        fg.connect(packetize, "pdus", decode, "pdus")
        # fg.connect(ns, 0, sync_long, 1).set_custom_buffer(gr.buffer_cpu_vmcirc_properties.make(gr.buffer_cpu_vmcirc_type.AUTO).set_buffer_size(buf_size))

        mtsched = mt.scheduler_mt("mtsched")
        tpsched = threadpool.scheduler_threadpool("threadpool", num_threads=args.nthreads)
          
        fg.add_scheduler(mtsched)
        fg.add_scheduler(tpsched)

        dconf = [ gr.domain_conf(mtsched, [ src, pre_sync, sync_short, sync_long, fft_blk, packetize ]),
                  gr.domain_conf(tpsched, [ decode ]) ]

        fg.partition(dconf)


    def sig_handler(sig=None, frame=None):
        fg.stop()
        fg.wait()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    print("starting ...")
    startt = time.time()
    fg.start()
    fg.wait()
    endt = time.time()

    print(f'[PROFILE_TIME]{endt-startt}[PROFILE_TIME]')

if __name__ == "__main__":
    main()
  
