yml_file = 'sync_short.yml'

sync_short_files = files(['sync_short_cpu.cc'])

if cuda_dep.found() and get_option('enable_cuda')
    # wifi_cu_sources += files('freq_correction.cu')
    sync_short_files += files(['sync_short_cuda.cc'])
endif

gen_sync_short_h = custom_target('gen_sync_short_cpu_h',
                        input : ['sync_short.yml'],
                        output : ['sync_short.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/wifi')

gen_sync_short_cc = custom_target('gen_sync_short_cpu_cc',
                        input : ['sync_short.yml'],
                        output : ['sync_short.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

wifi_deps += declare_dependency(sources : [gen_sync_short_h])
wifi_sources += [sync_short_files, gen_sync_short_cc]

if get_option('enable_python')
    gen_sync_short_pybind = custom_target('gen_sync_short_cpu_pybind',
                            input : ['sync_short.yml'],
                            output : ['sync_short_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    wifi_pybind_sources += gen_sync_short_pybind
    wifi_pybind_names += 'sync_short'
endif
