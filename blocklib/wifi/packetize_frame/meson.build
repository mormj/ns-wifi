srcs = files(['packetize_frame_cpu.cc'])

# if USE_CUDA
#     srcs += files('packetize_frame_cuda.cc')
#     wifi_cu_sources += files('packetize_frame_cuda.cu')
# endif

gen_srcs = custom_target('gen_packetize_frame_h',
                        input : ['packetize_frame.yml'],
                        output : ['packetize_frame.cc', 'packetize_frame.hh', 'packetize_frame_pybind.cc', 'wifi_packetize_frame.block.yml'],
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
    wifi_pybind_names += 'packetize_frame'
endif