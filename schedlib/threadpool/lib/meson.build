scheduler_threadpool_sources = [
    'thread_wrapper.cc',
    'scheduler_threadpool.cc',
]
scheduler_threadpool_deps = [newsched_runtime_dep, threads_dep, fmt_dep, pmtf_dep, python3_embed_dep]

link_args = ['-lnewsched-runtime']

incdir = include_directories('../include', '../include/gnuradio/schedulers/threadpool')
newsched_scheduler_threadpool_lib = library('newsched-scheduler-threadpool', 
    scheduler_threadpool_sources, include_directories : incdir, 
    install : true,
    link_language : 'cpp',
    link_args: link_args,
    dependencies : scheduler_threadpool_deps)

newsched_scheduler_threadpool_dep = declare_dependency(include_directories : incdir,
					   link_with : newsched_scheduler_threadpool_lib,
                       dependencies : scheduler_threadpool_deps )
