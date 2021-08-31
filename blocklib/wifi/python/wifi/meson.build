######################
#  Python Bindings ###
######################

# Manually created noblock sources
wifi_pybind_names = wifi_pybind_names

srcs = ['__init__.py']

foreach s: srcs
configure_file(copy: true,
    input: s,
    output: s
)
endforeach

d = {
  'blocks' : wifi_pybind_names,
  'module' : 'wifi',
  'imports' : ['newsched.gr']
}

gen_wifi_pybind = custom_target('gen_wifi_pybind',
                        output : ['wifi_pybind.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_module_pybind.py'),
                            '--blocks', d['blocks'],
                            '--imports', ' '.join(d['imports']),
                            '--module', d['module'],
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)      

wifi_pybind_sources += gen_wifi_pybind

newsched_blocklib_wifi_pybind = py3_inst.extension_module('wifi_python',
    wifi_pybind_sources, 
    dependencies : [newsched_blocklib_wifi_dep, python3_dep, pybind11_dep],
    link_language : 'cpp',
    install : true,
    install_dir : join_paths(py3_inst.get_install_dir(),'newsched','wifi')
)

newsched_blocklib_wifi_pybind_dep = declare_dependency(include_directories : incdir,
					   link_with : newsched_blocklib_wifi_pybind,
                       dependencies : wifi_deps)


# Target for pure python
py3_inst.install_sources(srcs, subdir : join_paths('newsched','wifi'))
