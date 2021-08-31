yml_file = 'frame_equalizer.yml'

frame_equalizer_files = files(['frame_equalizer_cpu.cc'])
if cuda_dep.found() and get_option('enable_cuda')
    wifi_cu_sources += files(
        ['frame_equalizer.cu',
         'multiply_const.cu',
         'equalize.cu',
         'constellation.cu'])
    frame_equalizer_files += files(['frame_equalizer_cuda.cc'])
endif

gen_frame_equalizer_h = custom_target('gen_frame_equalizer_cpu_h',
                        input : ['frame_equalizer.yml'],
                        output : ['frame_equalizer.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/wifi')

gen_frame_equalizer_cc = custom_target('gen_frame_equalizer_cpu_cc',
                        input : ['frame_equalizer.yml'],
                        output : ['frame_equalizer.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

wifi_deps += declare_dependency(sources : [gen_frame_equalizer_h])
wifi_sources += [frame_equalizer_files, gen_frame_equalizer_cc]

if get_option('enable_python')
    gen_frame_equalizer_pybind = custom_target('gen_frame_equalizer_cpu_pybind',
                            input : ['frame_equalizer.yml'],
                            output : ['frame_equalizer_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    wifi_pybind_sources += gen_frame_equalizer_pybind
    wifi_pybind_names += 'frame_equalizer'
endif
