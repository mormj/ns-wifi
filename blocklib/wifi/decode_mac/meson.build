srcs = files(['decode_mac_cpu.cc'])

# if USE_CUDA
#     srcs += files('decode_mac_cuda.cc')
#     wifi_cu_sources += files('decode_mac_cuda.cu')
# endif

gen_srcs = custom_target('gen_decode_mac_h',
                        input : ['decode_mac.yml'],
                        output : ['decode_mac.cc', 'decode_mac.hh', 'decode_mac_pybind.cc', 'wifi_decode_mac.block.yml'],
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
    wifi_pybind_names += 'decode_mac'
endif
