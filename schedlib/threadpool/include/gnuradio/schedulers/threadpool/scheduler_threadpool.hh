#include <gnuradio/block_group_properties.hh>
#include <gnuradio/domain.hh>
#include <gnuradio/graph_utils.hh>
#include <gnuradio/scheduler.hh>
#include <gnuradio/vmcircbuf.hh>

#include "thread_wrapper.hh"
namespace gr {
namespace schedulers {

/**
 * @brief Threadpool based scheduler
 * 
 * The Threadpool scheduler works by assigning received packets to a thread in
 * a threadpool, each of which will execute the same workflow
 * 
 * Current Limitations:
 * 1) Packet based input and output (no circ bufs)
 * 2) Single block with single input and single output (for now)
 * 
 */

class scheduler_threadpool : public scheduler
{
private:
    std::vector<threadpool::thread_wrapper::sptr> _threads;
    const int s_num_threads;
    const int s_queue_depth;
    std::map<nodeid_t, neighbor_interface_sptr> _block_thread_map;
    std::vector<block_group_properties> _block_groups;
    
public:
    typedef std::shared_ptr<scheduler_threadpool> sptr;
    static sptr make(const std::string name = "threadpool",
                     const unsigned int num_threads = 1,
                     const unsigned int queue_depth = 100)
    {
        return std::make_shared<scheduler_threadpool>(name, num_threads, queue_depth);
    }
    scheduler_threadpool(const std::string name = "threadpool",
                 const unsigned int num_threads = 1,
                     const unsigned int queue_depth = 100)
        : scheduler(name), s_num_threads(num_threads), s_queue_depth(queue_depth)
    {
        _default_buf_properties =
            vmcirc_buffer_properties::make(vmcirc_buffer_type::AUTO);
    }
    ~scheduler_threadpool(){};

    void push_message(scheduler_message_sptr msg);
    void add_block_group(const std::vector<block_sptr>& blocks,
                         const std::string& name = "",
                         const std::vector<unsigned int>& affinity_mask = {});

    /**
     * @brief Initialize the multi-threaded scheduler
     *
     * Creates a single-threaded scheduler for each block group, then for each block that
     * is not part of a block group
     *
     * @param fg subgraph assigned to this multi-threaded scheduler
     * @param fgmon sptr to flowgraph monitor object
     * @param block_sched_map for each block in this flowgraph, a map of neighboring
     * schedulers
     */
    void initialize(flat_graph_sptr fg, flowgraph_monitor_sptr fgmon);
    void start();
    void stop();
    void wait();
    void run();

};
} // namespace schedulers
} // namespace gr