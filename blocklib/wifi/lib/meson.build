incdir = include_directories(['../include/gnuradio/wifi','../include'])

wifi_deps += [newsched_runtime_dep, newsched_blocklib_digital_dep, volk_dep, fmt_dep, pmtf_dep]
wifi_sources += ['utils.cc', 
                 'constellations.cc']
wifi_sources += ['equalizer/base.cc',
                'equalizer/comb.cc',
                'equalizer/lms.cc',
                'equalizer/ls.cc',
                'equalizer/sta.cc',
                'viterbi_decoder/base.cc',
                # 'viterbi_decoder/viterbi_decoder_generic.cc',
                'viterbi_decoder/viterbi_decoder_x86.cc'
                ]

# This is ugly, and there HAS to be a better way to bring in dependencies
link_args = ['-lnewsched-runtime', '-lnewsched-blocklib-digital', '-lnewsched-blocklib-filter', '-lnewsched-blocklib-fft', '-lnewsched-blocklib-blocks']
block_cpp_args = ['-DHAVE_CPU']

wifi_cu_sources += ['freq_correction.cu']

if cuda_dep.found() #and get_option('enable_cuda')
     block_cpp_args += '-DHAVE_CUDA'

    newsched_blocklib_wifi_cu = library('newsched-blocklib-wifi-cu', 
        wifi_cu_sources, 
        include_directories : incdir, 
        install : true, 
        dependencies : [cuda_dep])

    newsched_blocklib_wifi_cu_dep = declare_dependency(include_directories : incdir,
                        link_with : newsched_blocklib_wifi_cu,
                        dependencies : cuda_dep)

     wifi_deps += [cuda_dep, cusp_dep, newsched_blocklib_wifi_cu_dep]
     link_args += '-lcusp'

endif

incdir = include_directories(['../include/gnuradio/wifi','../include'])
newsched_blocklib_wifi_lib = library('newsched-blocklib-wifi', 
    wifi_sources, 
    include_directories : incdir, 
    install : true,
    link_language: 'cpp',
    link_args : link_args,  # why is this necesary???
    dependencies : wifi_deps,
    cpp_args : block_cpp_args)

newsched_blocklib_wifi_dep = declare_dependency(include_directories : incdir,
					   link_with : newsched_blocklib_wifi_lib,
                       dependencies : wifi_deps)
