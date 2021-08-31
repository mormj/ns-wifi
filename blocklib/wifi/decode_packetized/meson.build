yml_file = 'decode_packetized.yml'

decode_packetized_files = files(['decode_packetized_cpu.cc'])

gen_decode_packetized_h = custom_target('gen_decode_packetized_cpu_h',
                        input : ['decode_packetized.yml'],
                        output : ['decode_packetized.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/wifi')

gen_decode_packetized_cc = custom_target('gen_decode_packetized_cpu_cc',
                        input : ['decode_packetized.yml'],
                        output : ['decode_packetized.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

wifi_deps += declare_dependency(sources : [gen_decode_packetized_h])
wifi_sources += [decode_packetized_files, gen_decode_packetized_cc]

if get_option('enable_python')
    gen_decode_packetized_pybind = custom_target('gen_decode_packetized_cpu_pybind',
                            input : ['decode_packetized.yml'],
                            output : ['decode_packetized_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    wifi_pybind_sources += gen_decode_packetized_pybind
    wifi_pybind_names += 'decode_packetized'
endif
