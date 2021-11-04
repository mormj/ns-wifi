yml_file = 'frame_equalizer.yml'

srcs = files(['frame_equalizer_cpu.cc'])
if USE_CUDA
    wifi_cu_sources += files(
        ['frame_equalizer.cu',
         'multiply_const.cu',
         'equalize.cu',
         'constellation.cu'])
    srcs += files(['frame_equalizer_cuda.cc'])
endif

gen_srcs = custom_target('gen_frame_equalizer_h',
                        input : ['frame_equalizer.yml'],
                        output : ['frame_equalizer.cc', 'frame_equalizer.hh', 'frame_equalizer_pybind.cc', 'wifi_frame_equalizer.block.yml'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output-cc', '@OUTPUT0@', 
                            '--output-hh', '@OUTPUT1@', 
                            '--output-pybind', '@OUTPUT2@', 
                            '--output-grc', '@OUTPUT3@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : [false, 'include/gnuradio/wifi',false,'share/gnuradio/grc/blocks'])

wifi_deps += declare_dependency(sources : [gen_srcs[1]] ) 
wifi_sources += [srcs, gen_srcs[0]]

if get_option('enable_python')                           
    wifi_pybind_sources += gen_srcs[2]
    wifi_pybind_names += 'frame_equalizer'
endif