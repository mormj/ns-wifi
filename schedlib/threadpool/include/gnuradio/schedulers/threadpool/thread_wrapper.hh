#pragma once

#include <gnuradio/block.hh>
#include <gnuradio/block_group_properties.hh>
#include <gnuradio/concurrent_queue.hh>
#include <gnuradio/flowgraph_monitor.hh>
#include <gnuradio/neighbor_interface.hh>
#include <gnuradio/neighbor_interface_info.hh>
#include <gnuradio/scheduler_message.hh>
#include <thread>

#include "threadpool/threadpool.h"
namespace gr {
namespace schedulers {
namespace threadpool {

struct threadpool_properties
{
    size_t num_threads = 1;
    size_t queue_depth = 100;
};

/**
 * @brief Wrapper for scheduler thread
 *
 * Creates the worker thread that will process work for all blocks in the graph assigned
 * to this scheduler.  This is the core of the single threaded scheduler.
 *
 */
class thread_wrapper : public neighbor_interface
{
private:
    /**
     * @brief Single message queue for all types of messages to this thread
     *
     */
    concurrent_queue<scheduler_message_sptr> msgq;
    std::thread d_thread;
    bool d_thread_stopped = false;

    int _id;
    block_group_properties d_block_group;
    std::vector<block_sptr> d_blocks;

    logger_sptr _logger;
    logger_sptr _debug_logger;

    flowgraph_monitor_sptr d_fgmon;

    message_port_sptr d_msgport;


public:
    typedef std::shared_ptr<thread_wrapper> sptr;

    static sptr make(int id,
                     block_group_properties bgp,
                     flowgraph_monitor_sptr fgmon,
                     threadpool_properties tpp)
    {
        return std::make_shared<thread_wrapper>(id, bgp, fgmon, tpp);
    }

    thread_wrapper(int id,
                   block_group_properties bgp,
                   flowgraph_monitor_sptr fgmon,
                   threadpool_properties tpp);
    int id() { return _id; }
    const std::string& name() { return d_block_group.name(); }

    void push_message(scheduler_message_sptr msg) { msgq.push(msg); }
    bool pop_message(scheduler_message_sptr& msg) { return msgq.pop(msg); }
    bool pop_message_nonblocking(scheduler_message_sptr& msg)
    {
        return msgq.try_pop(msg);
    }

    void start();
    void stop();
    void wait();
    void run();

    bool handle_work_notification();
    static void thread_body(thread_wrapper* top);

    bool _thrash = false;
    void set_thrash(bool thrash) { _thrash = thrash; }

    std::unique_ptr<threadpool> d_tp = nullptr;
};
} // namespace threadpool
} // namespace schedulers
} // namespace gr
