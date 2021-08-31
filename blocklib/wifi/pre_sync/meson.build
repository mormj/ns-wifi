yml_file = 'pre_sync.yml'

pre_sync_files = files(['pre_sync_cuda.cc'])
wifi_deps += [newsched_blocklib_blocks_dep]

if cuda_dep.found() and get_option('enable_cuda')
    wifi_cu_sources += files('presync.cu')
endif

gen_pre_sync_h = custom_target('gen_pre_sync_h',
                        input : ['pre_sync.yml'],
                        output : ['pre_sync.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/wifi')

gen_pre_sync_cc = custom_target('gen_pre_sync_cc',
                        input : ['pre_sync.yml'],
                        output : ['pre_sync.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

wifi_deps += declare_dependency(sources : [gen_pre_sync_h])
wifi_sources += [pre_sync_files, gen_pre_sync_cc]

if get_option('enable_python')
    gen_pre_sync_pybind = custom_target('gen_pre_sync_pybind',
                            input : ['pre_sync.yml'],
                            output : ['pre_sync_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    wifi_pybind_sources += gen_pre_sync_pybind
    wifi_pybind_names += 'pre_sync'
endif
