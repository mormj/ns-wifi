srcs = files(['sync_long_cpu.cc'])
if USE_CUDA
    wifi_cu_sources += files(['multiply.cu','remove_cp.cu'])
    srcs += files(['sync_long_cuda.cc'])
endif

gen_srcs = custom_target('gen_sync_long_h',
                        input : ['sync_long.yml'],
                        output : ['sync_long.cc', 'sync_long.hh', 'sync_long_pybind.cc', 'wifi_sync_long.block.yml'],
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
    wifi_pybind_names += 'sync_long'
endif