#include "thread_wrapper.hh"
#include <gnuradio/thread.hh>
#include <boost/format.hpp>
#include <thread>

namespace gr {
namespace schedulers {
namespace threadpool {

thread_wrapper::thread_wrapper(int id,
                               block_group_properties bgp,
                               flowgraph_monitor_sptr fgmon,
                               threadpool_properties tpp)
    : _id(id), d_block_group(bgp), d_blocks(bgp.blocks())
{
    _logger = logging::get_logger(bgp.name(), "default");
    _debug_logger = logging::get_logger(bgp.name() + "_dbg", "debug");

    d_fgmon = fgmon;

    GR_LOG_INFO(_logger, bgp.name());

    d_thread = std::thread(thread_body, this);
    d_msgport = this->d_blocks[0]->get_first_message_port(port_direction_t::INPUT);
    // auto port = this->d_blocks[0]->get_port(0, port_type_t::MESSAGE, port_direction_t::INPUT);
    // d_msgport = std::dynamic_pointer_cast<message_port>(
    //     this->d_blocks[0]->get_port(0, port_type_t::MESSAGE, port_direction_t::INPUT));

    std::vector<message_func_t> cb_funcs;
    auto lambda = [this](pmt::pmt_t msg) { return d_msgport->callback()(msg); };
    for (size_t i = 0; i < tpp.num_threads; i++) {
        cb_funcs.push_back((message_func_t)lambda);
    }
    d_tp = std::make_unique<threadpool>(tpp.num_threads, tpp.queue_depth, cb_funcs);
}

void thread_wrapper::start()
{
    for (auto& b : d_blocks) {
        b->start();
    }
    push_message(std::make_shared<scheduler_action>(scheduler_action_t::NOTIFY_ALL, 0));
}
void thread_wrapper::stop()
{
    d_thread_stopped = true;
    push_message(std::make_shared<scheduler_action>(scheduler_action_t::EXIT, 0));
    d_thread.join();
    for (auto& b : d_blocks) {
        b->stop();
    }
}
void thread_wrapper::wait()
{
    d_thread.join();
    for (auto& b : d_blocks) {
        b->done();
    }
}
void thread_wrapper::run()
{
    start();
    wait();
}

void thread_wrapper::thread_body(thread_wrapper* top)
{
    GR_LOG_INFO(top->_logger, "starting thread");

#if defined(_MSC_VER) || defined(__MINGW32__)
#include <windows.h>
    thread::set_thread_name(
        GetCurrentThread(),
        boost::str(boost::format("%s%d") % block->name() % block->unique_id()));
#else
    thread::set_thread_name(pthread_self(),
                            str(boost::format("%s%d") % top->d_block_group.name() %
                                top->d_block_group.blocks()[0]->id()));
#endif

    // Set thread affinity if it was set before fg was started.
    if (!top->d_block_group.processor_affinity().empty()) {
        std::cout << "setting affinity of thread " << thread::get_current_thread_id()
                  << " to " << top->d_block_group.processor_affinity()[0] << std::endl;
        gr::thread::thread_bind_to_processor(thread::get_current_thread_id(),
                                             top->d_block_group.processor_affinity());
    }

    // // Set thread priority if it was set before fg was started
    // if (block->thread_priority() > 0) {
    //     gr::thread::set_thread_priority(d->thread, block->thread_priority());
    // }

    bool blocking_queue = true;
    while (!top->d_thread_stopped) {

        scheduler_message_sptr msg;

        // try to pop messages off the queue
        bool valid = true;
        while (valid) {
            if (blocking_queue) {
                valid = top->pop_message(msg);
            } else {
                valid = top->pop_message_nonblocking(msg);
            }

            blocking_queue = false;

            if (valid) // this blocks
            {
                switch (msg->type()) {
                case scheduler_message_t::SCHEDULER_ACTION: {
                    // Notification that work needs to be done
                    // either from runtime or upstream or downstream or from self

                    auto action = std::static_pointer_cast<scheduler_action>(msg);
                    switch (action->action()) {
                    case scheduler_action_t::DONE:
                        // fgmon says that we need to be done, wrap it up
                        // each scheduler could handle this in a different way
                        gr_log_debug(top->_debug_logger,
                                     "fgm signaled DONE, pushing flushed");
                        top->d_fgmon->push_message(
                            fg_monitor_message(fg_monitor_message_t::FLUSHED, top->id()));
                        break;
                    case scheduler_action_t::EXIT:
                        gr_log_debug(top->_debug_logger,
                                     "fgm signaled EXIT, exiting thread");
                        // fgmon says that we need to be done, wrap it up
                        // each scheduler could handle this in a different way
                        top->d_thread_stopped = true;
                        break;

                    default:
                        break;
                    }
                    break;
                }
                case scheduler_message_t::MSGPORT_MESSAGE: {

                    auto m = std::static_pointer_cast<msgport_message>(msg);
                    // m->callback()(m->message());
                    top->d_tp->enqueue(m->message());

                    break;
                }
                default:
                    break;
                }
            }
        }

        blocking_queue = true;
    }

    std::cout << "clearing out the queue" << std::endl;

  while(top->d_tp->qsize() > 0)
  {
    std::cout << " qsize: " << top->d_tp->qsize() << std::endl;
    std::this_thread::sleep_for(100ms);
  }
  std::cout << "queue cleared" << std::endl;

}
} // namespace threadpool
} // namespace schedulers
} // namespace gr
