incdir = include_directories('../include')

###################################################
#    QA
###################################################

if get_option('enable_testing')
    env = environment()
    env.prepend('LD_LIBRARY_PATH', join_paths( meson.build_root(),'schedulers','threadpool','lib'))
    env.prepend('PYTHONPATH', join_paths(meson.build_root(),'python'))

endif
