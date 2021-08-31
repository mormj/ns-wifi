#include <gnuradio/schedulers/threadpool/scheduler_threadpool.hh>

namespace gr {
namespace schedulers {

void scheduler_threadpool::push_message(scheduler_message_sptr msg)
{
    // Use 0 for blkid all threads
    if (msg->blkid() == 0) {
        for (auto element : _block_thread_map) {
            auto thd = element.second;
            thd->push_message(msg);
        }
    } else {
        _block_thread_map[msg->blkid()]->push_message(msg);
    }
}

void scheduler_threadpool::add_block_group(const std::vector<block_sptr>& blocks,
                                           const std::string& name,
                                           const std::vector<unsigned int>& affinity_mask)
{
    _block_groups.push_back(
        std::move(block_group_properties(blocks, name, affinity_mask)));
}

void scheduler_threadpool::initialize(flat_graph_sptr fg, flowgraph_monitor_sptr fgmon)
{
    int blk_cnt = 0;
    for (auto& b : fg->calc_used_blocks()) {
        b->set_scheduler(base());
        blk_cnt++;
    }
    if (blk_cnt > 1) {
        throw std::runtime_error(
            "Currently the scheduler only works with 1 block flowgraphs :/");
    }

    auto blocks = fg->calc_used_blocks();
    auto t = threadpool::thread_wrapper::make(
        id(), block_group_properties{ blocks }, fgmon, { s_num_threads, s_queue_depth });
    _threads.push_back(t);

    for (auto& b : blocks) {

        for (auto& p : b->all_ports()) {
            p->set_parent_intf(t); // give a shared pointer to the scheduler class
        }
        _block_thread_map[b->id()] = t;
    }
}

void scheduler_threadpool::start()
{
    for (const auto& thd : _threads) {
        thd->start();
    }
}
void scheduler_threadpool::stop()
{
    for (const auto& thd : _threads) {
        thd->stop();
    }
}
void scheduler_threadpool::wait()
{
    for (const auto& thd : _threads) {
        thd->wait();
    }
}
void scheduler_threadpool::run()
{
    for (const auto& thd : _threads) {
        thd->start();
    }
    for (const auto& thd : _threads) {
        thd->wait();
    }
}

} // namespace schedulers
} // namespace gr


// External plugin interface for instantiating out of tree schedulers
// TODO: name, version, other info methods
extern "C" {
std::shared_ptr<gr::scheduler> factory(const std::string& name = "threadpool",
                                       size_t num_threads = 1,
                                       size_t queue_depth = 100)
{
    return gr::schedulers::scheduler_threadpool::make(name, num_threads, queue_depth);
}
}