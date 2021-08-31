#!/usr/bin/env python3

from newsched import gr, blocks, streamops, fileio, math, wifi, filter, fft
import numpy as np
import argparse
import time
import signal, sys

def argParse():
    """Parses commandline args."""
    desc='Scrape the doxygen generated xml for docstrings to insert into python bindings'
    parser = argparse.ArgumentParser(description=desc)
    
    parser.add_argument("filename")

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

    if 0: 
        src = fileio.file_source(gr.sizeof_gr_complex, args.filename, False)
        # src = fileio.file_source(gr.sizeof_float, args.filename, False)
        sync_short = wifi.sync_short(0.56, 2, True, False)
        sync_long = wifi.sync_long(sync_length, True, False)
        # hd = blocks.head(gr.sizeof_gr_complex, 1000000000)

        mult = math.multiply_cc(2)
        macc = filter.moving_average_cc(window_size, 1, 4000, 1) #, impl=filter.moving_average_cc.cuda)
        maff = filter.moving_average_ff(window_size  + 16, 1, 4000, 1) #, impl=filter.moving_average_ff.cuda)
        
        div = math.divide_ff(2)
        dly0 = blocks.delay(gr.sizeof_gr_complex*1, 16)
        dly1 = blocks.delay(gr.sizeof_gr_complex*1, sync_length)
        conj = math.conjugate()
        cpmgsq = math.complex_to_mag_squared(1)
        cpmg = math.complex_to_mag(1)
        fft_blk = fft.fft_cc_fwd(64, np.ones(64), True)
        frameeq = wifi.frame_equalizer(0,2412e6,10e6,False, False)
        # snk = fileio.file_sink(gr.sizeof_gr_complex, "/tmp/wifi_out.fc32")
        # snk = fileio.file_sink(gr.sizeof_float, "wifi_out.fc32")
        snk = fileio.file_sink(48*gr.sizeof_char, "/tmp/wifi_out.fc32")

        snk1 = fileio.file_sink(gr.sizeof_gr_complex, "/tmp/dbg_out1.fc32")
        snk2 = fileio.file_sink(gr.sizeof_gr_complex, "/tmp/dbg_out2.fc32")
        snk3 = fileio.file_sink(gr.sizeof_float, "/tmp/dbg_out3.fc32")
        snk4 = fileio.file_sink(gr.sizeof_float, "/tmp/dbg_out4.fc32")
        snk5 = fileio.file_sink(gr.sizeof_float, "/tmp/dbg_out5.fc32")

        # fg.connect(src,0,hd,0)
        fg.connect(src, 0, dly0, 0)
        fg.connect(src, 0, mult, 0)
        fg.connect(src, 0, cpmgsq, 0)
        
        fg.connect(dly0, 0, conj, 0)
        fg.connect(conj, 0, mult, 1)
        fg.connect(cpmgsq, 0, maff, 0) #.set_custom_buffer(gr.cuda_buffer_sm_properties.make(gr.cuda_buffer_sm_type.H2D)) #.set_buffer_size(65536*4))
        fg.connect(mult, 0, macc, 0) #.set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.H2D))
        fg.connect(macc, 0, cpmg, 0) #.set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H))
        fg.connect(sync_short, 0, dly1, 0)
        fg.connect(sync_short, 0, sync_long, 0)
        fg.connect(dly1, 0, sync_long, 1)
        fg.connect(cpmg, 0, div, 0)
        fg.connect(maff, 0, div, 1) #.set_custom_buffer(gr.cuda_buffer_sm_properties.make(gr.cuda_buffer_sm_type.D2H))

        fg.connect(dly0, 0, sync_short, 0)
        fg.connect(macc, 0, sync_short, 1) #.set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H))
        fg.connect(div, 0, sync_short, 2)
        # fg.connect(sync_long, 0, snk, 0)
        # fg.connect(sync_short, 0, snk, 0)
        fg.connect(sync_long, 0, fft_blk, 0)
        fg.connect(fft_blk, 0, frameeq, 0)
        # fg.connect(fft_blk, 0, snk, 0)
        fg.connect(frameeq, 0, snk, 0)


        # src1 = fileio.file_source(gr.sizeof_gr_complex, '/tmp/sync_short_in_0.fc32', False)
        # src2 = fileio.file_source(gr.sizeof_gr_complex,  '/tmp/sync_short_in_1.fc32', False)
        # src3 = fileio.file_source(gr.sizeof_float,  '/tmp/sync_short_in_2.fc32', False)

        # fg.connect(src1, 0, sync_short, 0)
        # fg.connect(src2, 0, sync_short, 1)
        # fg.connect(src3, 0, sync_short, 2)

        # fg.connect(dly0, 0, snk1, 0)
        # fg.connect(macc, 0, snk2, 0)
        # fg.connect(div, 0, snk3, 0)
        # fg.connect(maff, 0, snk4, 0)
        # fg.connect(cpmgsq, 0, snk5, 0)

        # fg.connect(src,0,maff,0)
        # fg.connect(maff,0,snk,0)
        # fg.connect(div,0,snk,0)
        # fg.connect(mult,0,snk,0)
        # fg.connect(sync_short, 0, snk, 0)  
    elif 0:
        src = fileio.file_source(gr.sizeof_gr_complex, args.filename, False)
        # src = fileio.file_source(gr.sizeof_float, args.filename, False)
        sync_short = wifi.sync_short(0.56, 2, True, False)
        sync_long = wifi.sync_long(sync_length, True, False)
        # hd = blocks.head(gr.sizeof_gr_complex, 1000000000)
        # self.wifi_parse_mac_0 = wifi.parse_mac(True, True)
        # self.frame_equalizer_0 = wifi.frame_equalizer(chan_est, freq, samp_rate, True, False)
        # self.wifi_decode_mac_0 = wifi.decode_mac(True, False)
        # self.fft_vxx_0 = fft.fft_vcc(64, True, window.rectangular(64), True, 1)
        # self.blocks_stream_to_vector_0 = blocks.stream_to_vector(gr.sizeof_gr_complex*1, 64)
        mult = math.multiply_cc(2)
        macc = filter.moving_average_cc(window_size, 1, 4000, 1) #, impl=filter.moving_average_cc.cuda)
        maff = filter.moving_average_ff(window_size  + 16, 1, 4000, 1, impl=filter.moving_average_ff.cuda)
        # maff = filter.moving_average_ff(16, 1, 40, 1)
        div = math.divide_ff(2)
        dly0 = blocks.delay(gr.sizeof_gr_complex*1, 16)
        dly1 = blocks.delay(gr.sizeof_gr_complex*1, sync_length)
        conj = math.conjugate()
        cpmgsq = math.complex_to_mag_squared(1)
        cpmg = math.complex_to_mag(1)
        fft_blk = fft.fft_cc_fwd(64, np.ones(64), True)
        frameeq = wifi.frame_equalizer(0,2412e6,20e6,False, False)
        snk = fileio.file_sink(gr.sizeof_gr_complex, "/tmp/wifi_out.fc32")
        # snk = fileio.file_sink(gr.sizeof_float, "wifi_out.fc32")
        # snk = fileio.file_sink(48*gr.sizeof_char, "/tmp/wifi_out.fc32")

        snk1 = fileio.file_sink(gr.sizeof_gr_complex, "/tmp/dbg_out1.fc32")
        snk2 = fileio.file_sink(gr.sizeof_gr_complex, "/tmp/dbg_out2.fc32")
        snk3 = fileio.file_sink(gr.sizeof_float, "/tmp/dbg_out3.fc32")
        snk4 = fileio.file_sink(gr.sizeof_float, "/tmp/dbg_out4.fc32")
        snk5 = fileio.file_sink(gr.sizeof_float, "/tmp/dbg_out5.fc32")

        # fg.connect(src,0,hd,0)
        fg.connect(src, 0, dly0, 0)
        fg.connect(src, 0, mult, 0)
        fg.connect(src, 0, cpmgsq, 0)
        
        fg.connect(dly0, 0, conj, 0)
        fg.connect(conj, 0, mult, 1)
        fg.connect(cpmgsq, 0, maff, 0).set_custom_buffer(gr.cuda_buffer_sm_properties.make(gr.cuda_buffer_sm_type.H2D)) #.set_buffer_size(65536*4))
        fg.connect(mult, 0, macc, 0) #.set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.H2D))
        fg.connect(macc, 0, cpmg, 0) #.set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H))
        # fg.connect(sync_short, 0, dly1, 0)
        # fg.connect(sync_short, 0, sync_long, 0)
        # fg.connect(dly1, 0, sync_long, 1)
        fg.connect(cpmg, 0, div, 0)
        fg.connect(maff, 0, div, 1).set_custom_buffer(gr.cuda_buffer_sm_properties.make(gr.cuda_buffer_sm_type.D2H))

        fg.connect(dly0, 0, sync_short, 0)
        fg.connect(macc, 0, sync_short, 1) #.set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H))
        fg.connect(div, 0, sync_short, 2)
        # fg.connect(sync_long, 0, snk, 0)
        fg.connect(sync_short, 0, snk, 0)
        # fg.connect(sync_long, 0, fft_blk, 0)
        # fg.connect(fft_blk, 0, frameeq, 0)
        # fg.connect(fft_blk, 0, snk, 0)
        # fg.connect(frameeq, 0, snk, 0)


        # src1 = fileio.file_source(gr.sizeof_gr_complex, '/tmp/sync_short_in_0.fc32', False)
        # src2 = fileio.file_source(gr.sizeof_gr_complex,  '/tmp/sync_short_in_1.fc32', False)
        # src3 = fileio.file_source(gr.sizeof_float,  '/tmp/sync_short_in_2.fc32', False)

        # fg.connect(src1, 0, sync_short, 0)
        # fg.connect(src2, 0, sync_short, 1)
        # fg.connect(src3, 0, sync_short, 2)

        # fg.connect(dly0, 0, snk1, 0)
        # fg.connect(macc, 0, snk2, 0)
        # fg.connect(div, 0, snk3, 0)
        # fg.connect(maff, 0, snk4, 0)
        # fg.connect(cpmgsq, 0, snk5, 0)

        # fg.connect(src,0,maff,0)
        # fg.connect(maff,0,snk,0)
        # fg.connect(div,0,snk,0)
        # fg.connect(mult,0,snk,0)
        # fg.connect(sync_short, 0, snk, 0)

    elif 0: 
        src = fileio.file_source(gr.sizeof_gr_complex, args.filename, False)
        # src = fileio.file_source(gr.sizeof_float, args.filename, False)
        sync_short = wifi.sync_short(0.56, 2, True, False)

        mult = math.multiply_cc(2)
        macc = filter.moving_average_cc(window_size, 1, 4000, 1) #, impl=filter.moving_average_cc.cuda)
        maff = filter.moving_average_ff(window_size  + 16, 1, 4000, 1) #, impl=filter.moving_average_ff.cuda)
        
        div = math.divide_ff(2)
        dly0 = blocks.delay(gr.sizeof_gr_complex*1, 16)
        conj = math.conjugate()
        cpmgsq = math.complex_to_mag_squared(1)
        cpmg = math.complex_to_mag(1)
        fft_blk = fft.fft_cc_fwd(64, np.ones(64), True)
        frameeq = wifi.frame_equalizer(0,2412e6,10e6,False, False)
        # snk = fileio.file_sink(gr.sizeof_gr_complex, "/tmp/wifi_out.fc32")
        # snk = fileio.file_sink(gr.sizeof_float, "wifi_out.fc32")
        snk = fileio.file_sink(gr.sizeof_gr_complex, "/tmp/wifi_out_cpu.fc32")

        # fg.connect(src,0,hd,0)
        fg.connect(src, 0, dly0, 0)
        fg.connect(src, 0, mult, 0)
        fg.connect(src, 0, cpmgsq, 0)
        
        fg.connect(dly0, 0, conj, 0)
        fg.connect(conj, 0, mult, 1)
        fg.connect(cpmgsq, 0, maff, 0) #.set_custom_buffer(gr.cuda_buffer_sm_properties.make(gr.cuda_buffer_sm_type.H2D)) #.set_buffer_size(65536*4))
        fg.connect(mult, 0, macc, 0) #.set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.H2D))
        fg.connect(macc, 0, cpmg, 0) #.set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H))
        fg.connect(sync_short, 0, snk, 0)

        fg.connect(cpmg, 0, div, 0)
        fg.connect(maff, 0, div, 1) #.set_custom_buffer(gr.cuda_buffer_sm_properties.make(gr.cuda_buffer_sm_type.D2H))

        fg.connect(dly0, 0, sync_short, 0)
        fg.connect(macc, 0, sync_short, 1) #.set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H))
        fg.connect(div, 0, sync_short, 2)

    elif 1: # through sync_short
        src = fileio.file_source(gr.sizeof_gr_complex, args.filename, False)
        pre_sync = wifi.pre_sync(48,1024*1024*2, impl=wifi.pre_sync.cuda)
        sync_short = wifi.sync_short(0.56, 2, True, False, impl=wifi.sync_short.cuda)
        sync_long = wifi.sync_long(sync_length, True, False, impl=wifi.sync_long.cuda)
        # ns = blocks.null_source(gr.sizeof_gr_complex)

        # snk = fileio.file_sink(gr.sizeof_gr_complex, "/tmp/wifi_out.fc32")
        # snk = blocks.null_sink(gr.sizeof_gr_complex)
        snk = blocks.null_sink(64*gr.sizeof_gr_complex)

        buf_size = 2048*1024
        # fg.connect(src,0,hd,0)
        fg.connect(src, 0, pre_sync, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.H2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 0, sync_short, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 1, sync_short, 1).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 2, sync_short, 2).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        # fg.connect(sync_short, 0, snk, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
        # fg.connect(sync_long, 0, snk, 0)
        fg.connect(sync_short, 0, sync_long, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(sync_long, 0, snk, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
        # fg.connect(ns, 0, sync_long, 1).set_custom_buffer(gr.vmcirc_buffer_properties.make(gr.vmcirc_buffer_type.AUTO).set_buffer_size(buf_size))
    elif 0:
        src = fileio.file_source(gr.sizeof_gr_complex, args.filename, False)
        pre_sync = wifi.pre_sync(48,1024*1024*2, impl=wifi.pre_sync.cuda)
        snk1 = fileio.file_sink(gr.sizeof_gr_complex, "/tmp/ps1.fc32")
        snk2 = fileio.file_sink(gr.sizeof_gr_complex, "/tmp/ps2.fc32")
        snk3 = fileio.file_sink(gr.sizeof_float, "/tmp/ps3.f32")

        buf_size = 1024*1024
        fg.connect(src, 0, pre_sync, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.H2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 0, snk1, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
        fg.connect(pre_sync, 1, snk2, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
        fg.connect(pre_sync, 2, snk3, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
    elif 0: # CUDA through fft
        src = fileio.file_source(gr.sizeof_gr_complex, args.filename, False)
        pre_sync = wifi.pre_sync(48,1024*1024*8, impl=wifi.pre_sync.cuda)
        sync_short = wifi.sync_short(0.56, 2, True, False, impl=wifi.sync_short.cuda)
        sync_long = wifi.sync_long(sync_length, True, False, impl=wifi.sync_long.cuda)
        # sync_short = wifi.sync_short(0.56, 2, True, False, impl=wifi.sync_short.cpu)
        # sync_long = wifi.sync_long(sync_length, True, False, impl=wifi.sync_long.cpu)
        dly1 = blocks.delay(gr.sizeof_gr_complex*1, sync_length)
        ns = blocks.null_source(gr.sizeof_gr_complex)
        fft_blk = fft.fft_cc_fwd(64, np.ones(64), True, impl=fft.fft_cc_fwd.cuda)
        # fft_blk = fft.fft_cc_fwd(64, np.ones(64), True, impl=fft.fft_cc_fwd.cpu)
        # frameeq = wifi.frame_equalizer(0,2412e6,10e6,False, False, impl=wifi.frame_equalizer.cuda)
        frameeq = wifi.frame_equalizer(0,2412e6,10e6,False, False, impl=wifi.frame_equalizer.cpu)
        
        snk = fileio.file_sink(48*gr.sizeof_char, "/tmp/wifi_out.fc32")

        buf_size = 1024*1024
        # fg.connect(src,0,hd,0)
        fg.connect(src, 0, pre_sync, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.H2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 0, sync_short, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 1, sync_short, 1).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 2, sync_short, 2).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        # fg.connect(sync_short,0, dly1,0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
        # fg.connect(sync_long, 0, snk, 0)
        fg.connect(sync_short, 0, sync_long, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(sync_long, 0, fft_blk, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        # fg.connect(dly1, 0, sync_long, 1) #.set_custom_buffer(gr.vmcirc_buffer_properties.make(gr.vmcirc_buffer_type.AUTO).set_buffer_size(buf_size))
        fg.connect(ns, 0, sync_long, 1) #.set_custom_buffer(gr.vmcirc_buffer_properties.make(gr.vmcirc_buffer_type.AUTO).set_buffer_size(buf_size))
        fg.connect(fft_blk, 0, frameeq, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
        fg.connect(frameeq, 0, snk, 0) #.set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
    elif 1: # CUDA through sync_long
        src = fileio.file_source(gr.sizeof_gr_complex, args.filename, False)
        pre_sync = wifi.pre_sync(48,1024*1024*8, impl=wifi.pre_sync.cuda)
        sync_short = wifi.sync_short(0.56, 2, True, False, impl=wifi.sync_short.cuda)
        sync_long = wifi.sync_long(sync_length, True, False, impl=wifi.sync_long.cuda)
        # sync_short = wifi.sync_short(0.56, 2, True, False, impl=wifi.sync_short.cpu)
        # sync_long = wifi.sync_long(sync_length, True, False, impl=wifi.sync_long.cpu)
        dly1 = blocks.delay(gr.sizeof_gr_complex*1, sync_length)
        # ns = blocks.null_source(gr.sizeof_gr_complex)
        fft_blk = fft.fft_cc_fwd(64, np.ones(64), True, impl=fft.fft_cc_fwd.cuda)
        # fft_blk = fft.fft_cc_fwd(64, np.ones(64), True, impl=fft.fft_cc_fwd.cpu)
        # frameeq = wifi.frame_equalizer(0,2412e6,10e6,False, False, impl=wifi.frame_equalizer.cuda)
        frameeq = wifi.frame_equalizer(0,2412e6,20e6,False, False, impl=wifi.frame_equalizer.cpu)
        
        snk = fileio.file_sink(48*gr.sizeof_char, "/tmp/wifi_out.fc32")

        # decode_mac = wifi.wifi_decode_mac(False, False)

        buf_size = 1024*1024
        # fg.connect(src,0,hd,0)
        fg.connect(src, 0, pre_sync, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.H2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 0, sync_short, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 1, sync_short, 1).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 2, sync_short, 2).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        # fg.connect(sync_short,0, dly1,0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
        # fg.connect(sync_long, 0, snk, 0)
        fg.connect(sync_short, 0, sync_long, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(sync_long, 0, fft_blk, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        # fg.connect(dly1, 0, sync_long, 1) #.set_custom_buffer(gr.vmcirc_buffer_properties.make(gr.vmcirc_buffer_type.AUTO).set_buffer_size(buf_size))
        fg.connect(fft_blk, 0, frameeq, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
        # fg.connect(fft_blk, 0, frameeq, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(frameeq, 0, snk, 0) #.set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
        # fg.connect(frameeq, 0, decode_mac, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
        # fg.connect(frameeq, 0, decode_mac, 0) #.set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))

        
    else: # CUDA through frame_equalizer and decode
        src = fileio.file_source(gr.sizeof_gr_complex, args.filename, False)
        pre_sync = wifi.pre_sync(48,1024*1024*8, impl=wifi.pre_sync.cuda)
        sync_short = wifi.sync_short(0.56, 2, True, False, impl=wifi.sync_short.cuda)
        sync_long = wifi.sync_long(sync_length, True, False, impl=wifi.sync_long.cuda)
        fft_blk = fft.fft_cc_fwd(64, np.ones(64), True, impl=fft.fft_cc_fwd.cuda)
        # fft_blk = fft.fft_cc_fwd(64, np.ones(64), True, impl=fft.fft_cc_fwd.cpu)
        frameeq = wifi.frame_equalizer(0,2412e6,20e6,False, False, impl=wifi.frame_equalizer.cuda)
        # frameeq = wifi.frame_equalizer(0,2412e6,10e6,False, False, impl=wifi.frame_equalizer.cpu)
        decode_mac = wifi.wifi_decode_mac(False, False)

        snk = fileio.file_sink(48*gr.sizeof_char, "/tmp/wifi_out.fc32")

        buf_size = 1024*1024
        # fg.connect(src,0,hd,0)
        fg.connect(src, 0, pre_sync, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.H2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 0, sync_short, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 1, sync_short, 1).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(pre_sync, 2, sync_short, 2).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        # fg.connect(sync_short,0, dly1,0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
        # fg.connect(sync_long, 0, snk, 0)
        fg.connect(sync_short, 0, sync_long, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(sync_long, 0, fft_blk, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        fg.connect(fft_blk, 0, frameeq, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2D).set_buffer_size(buf_size))
        # fg.connect(frameeq, 0, snk, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))
        fg.connect(frameeq, 0, decode_mac, 0).set_custom_buffer(gr.cuda_buffer_properties.make(gr.cuda_buffer_type.D2H).set_buffer_size(buf_size))




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
  
