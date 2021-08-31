subdir('include/gnuradio/wifi')

wifi_sources = []
wifi_cu_sources = []
wifi_pybind_sources = []
wifi_pybind_names = []
wifi_deps = [newsched_blocklib_filter_dep, newsched_blocklib_fft_dep, newsched_blocklib_digital_dep]
# Individual block subdirectories
subdir('pre_sync')
subdir('sync_short')
subdir('sync_long')
subdir('frame_equalizer')
subdir('packetize_frame')
subdir('decode_mac')
subdir('decode_packetized')

subdir('lib')
if (get_option('enable_python'))
    subdir('python/wifi')
endif

subdir('test')
