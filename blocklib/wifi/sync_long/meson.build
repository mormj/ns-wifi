yml_file = 'sync_long.yml'

sync_long_files = files(['sync_long_cpu.cc'])
if cuda_dep.found() and get_option('enable_cuda')
    wifi_cu_sources += files(['multiply.cu','remove_cp.cu'])
    sync_long_files += files(['sync_long_cuda.cc'])
endif

gen_sync_long_h = custom_target('gen_sync_long_cpu_h',
                        input : ['sync_long.yml'],
                        output : ['sync_long.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/wifi')

gen_sync_long_cc = custom_target('gen_sync_long_cpu_cc',
                        input : ['sync_long.yml'],
                        output : ['sync_long.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

wifi_deps += declare_dependency(sources : [gen_sync_long_h])
wifi_sources += [sync_long_files, gen_sync_long_cc]

if get_option('enable_python')
    gen_sync_long_pybind = custom_target('gen_sync_long_cpu_pybind',
                            input : ['sync_long.yml'],
                            output : ['sync_long_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    wifi_pybind_sources += gen_sync_long_pybind
    wifi_pybind_names += 'sync_long'
endif
