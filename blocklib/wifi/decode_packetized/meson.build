srcs = files(['decode_packetized_cpu.cc'])

# if USE_CUDA
#     srcs += files('decode_packetized_cuda.cc')
#     wifi_cu_sources += files('decode_packetized_cuda.cu')
# endif

gen_srcs = custom_target('gen_decode_packetized_h',
                        input : ['decode_packetized.yml'],
                        output : ['decode_packetized.cc', 'decode_packetized.hh', 'decode_packetized_pybind.cc', 'wifi_decode_packetized.block.yml'],
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
    wifi_pybind_names += 'decode_packetized'
endif