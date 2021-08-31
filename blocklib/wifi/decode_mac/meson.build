yml_file = 'decode_mac.yml'

decode_mac_files = files(['decode_mac_cpu.cc'])

gen_decode_mac_h = custom_target('gen_decode_mac_cpu_h',
                        input : ['decode_mac.yml'],
                        output : ['decode_mac.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/wifi')

gen_decode_mac_cc = custom_target('gen_decode_mac_cpu_cc',
                        input : ['decode_mac.yml'],
                        output : ['decode_mac.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

wifi_deps += declare_dependency(sources : [gen_decode_mac_h])
wifi_sources += [decode_mac_files, gen_decode_mac_cc]

if get_option('enable_python')
    gen_decode_mac_pybind = custom_target('gen_decode_mac_cpu_pybind',
                            input : ['decode_mac.yml'],
                            output : ['decode_mac_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    wifi_pybind_sources += gen_decode_mac_pybind
    wifi_pybind_names += 'decode_mac'
endif
