# SPDX-FileCopyrightText: 2020 Josh Morman 
#

project('newsched-wifi', 'cpp', 
  version : '0.0.0',
  license : 'GPLv3',
  default_options : ['cpp_std=c++17'])

cc = meson.get_compiler('cpp')
rt_dep = cc.find_library('rt', required : false)
libdl_dep = cc.find_library('dl')

c_available = add_languages('c', required : true)
cuda_available = add_languages('cuda', required : false)

boost_dep = dependency('boost', modules : ['program_options','thread'], version : '>=1.65')
volk_dep = dependency('volk', version : '>=2.2')
yaml_dep = dependency('yaml-cpp', version : '>=0.6')
fmt_dep = dependency('fmt', method: 'cmake', modules: ['fmt::fmt'])
spdlog_dep = dependency('spdlog', method: 'cmake', modules : ['spdlog::spdlog'])
python3_dep = dependency('python3', required : get_option('enable_python'))

# Import python3 meson module which is used to find the Python dependencies.
py3_inst = import('python').find_installation('python3')

pybind11_dep = dependency('pybind11', required : get_option('enable_python'))
threads_dep = dependency('threads')
gtest_dep = dependency('gtest', main : true, version : '>=1.10', required : get_option('enable_testing'))
cuda_dep = dependency('cuda', version : '>=10.1', required : cuda_available and get_option('enable_cuda'), modules : ['cufft'])
cusp_dep = dependency('cusp')
USE_CUDA = cuda_available and get_option('enable_cuda')

flatc = find_program('flatc', version : '>=1.12')

prefix = get_option('prefix')
# Escape spaces
prefix_parts = prefix.split(' ')
prefix = '\ '.join(prefix_parts)
share_dir = join_paths(prefix,'share')

# newsched_dep = dependency('newsched', method: 'cmake')
# TODO - do this with subcomponents
pmt_dep = dependency('newsched-pmt')
newsched_runtime_dep = dependency('newsched-runtime')
newsched_blocklib_filter_dep = dependency('newsched-filter')
newsched_blocklib_fft_dep = dependency('newsched-fft')
newsched_blocklib_digital_dep = dependency('newsched-digital')
newsched_blocklib_blocks_dep = dependency('newsched-blocks')

subdir('blocklib')
subdir('schedlib')
# subdir('docs')
# if (get_option('enable_python'))
#   subdir('python/wifi')
# endif
