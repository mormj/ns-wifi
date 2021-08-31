yml_file = 'packetize_frame.yml'

packetize_frame_files = files(['packetize_frame_cpu.cc'])
# if cuda_dep.found() and get_option('enable_cuda')
#     wifi_cu_sources += files(
#         ['packetize_frame.cu',
#          'multiply_const.cu',
#          'equalize.cu',
#          'constellation.cu'])
#     packetize_frame_files += files(['packetize_frame_cuda.cc'])
# endif

gen_packetize_frame_h = custom_target('gen_packetize_frame_cpu_h',
                        input : ['packetize_frame.yml'],
                        output : ['packetize_frame.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/wifi')

gen_packetize_frame_cc = custom_target('gen_packetize_frame_cpu_cc',
                        input : ['packetize_frame.yml'],
                        output : ['packetize_frame.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

wifi_deps += declare_dependency(sources : [gen_packetize_frame_h])
wifi_sources += [packetize_frame_files, gen_packetize_frame_cc]

if get_option('enable_python')
    gen_packetize_frame_pybind = custom_target('gen_packetize_frame_cpu_pybind',
                            input : ['packetize_frame.yml'],
                            output : ['packetize_frame_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    wifi_pybind_sources += gen_packetize_frame_pybind
    wifi_pybind_names += 'packetize_frame'
endif
