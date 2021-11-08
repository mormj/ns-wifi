/*********************************************************
 *
 *  Copyright (C) 2014 by Vitaliy Vitsentiy
 *  Modified 2021 by Josh Morman
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *********************************************************/
#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "detail.h"

#include <pmtf/wrap.hpp>
#include <gnuradio/thread.hh>
using namespace std::chrono_literals;

namespace gr {
namespace schedulers {
namespace threadpool {

typedef std::function<void(pmtf::wrap)> message_func_t;

class threadpool
{
public:
    std::vector<std::shared_ptr<std::atomic<bool>>> flags;
    threadpool(unsigned numthreads, unsigned queuedepth, const std::vector<message_func_t> message_funcs)
    {
        m_numthreads = numthreads;
        m_queuedepth = queuedepth;

        this->threads.resize(numthreads);
        this->flags.resize(numthreads);

        for (unsigned i = 0; i < numthreads; i++) {
            this->flags[i] = std::make_shared<std::atomic<bool>>(false);
            std::shared_ptr<std::atomic<bool>> flag(
                this->flags[i]); // a copy of the shared ptr to the flag
            auto f = [this, i, flag, message_funcs /* a copy of the shared ptr to the flag */]() {
                gr::thread::set_thread_name(pthread_self(),
                                        std::string("tp") + std::to_string(i));

                std::atomic<bool>& _flag = *flag;
                pmtf::wrap _p;

                bool isPop = this->q.pop(_p);
                // burst_worker b(i);
                while (true) {
                    while (isPop) { // if there is anything in the queue
                        message_funcs[i](_p);
                        // std::this_thread::sleep_for(2000ms);
                        if (_flag) {
                            return; // the thread is wanted to stop, return even if the
                                    // queue is not empty yet
                        } else
                            isPop = this->q.pop(_p);
                    }
                    // the queue is empty here, wait for the next command
                    std::unique_lock<std::mutex> lock(this->m_mutex);
                    ++this->nWaiting;
                    this->cv.wait(lock, [this, &_p, &isPop, &_flag]() {
                        isPop = this->q.pop(_p);
                        return isPop || this->isDone || _flag;
                    });
                    --this->nWaiting;
                    // if (!isPop) {
                    //     std::cout << "return2 " << i << std::endl;
                    //     return;  // if the queue is empty and this->isDone == true or
                    //     *flag then return
                    // }
                }
            };
            this->threads[i].reset(new std::thread(f));
        }
    }
    ~threadpool() { std::cout << "threadpool: " << std::endl; };
    void enqueue(pmtf::wrap p)
    {

        this->q.push(p);
        // std::cout << "enqueuing ... " << this->q.qsize() << std::endl;
        std::unique_lock<std::mutex> lock(this->m_mutex);
        this->cv.notify_one();
    }
    pmtf::wrap dequeue()
    {
        pmtf::wrap r;
        this->oq.pop(r);
        return r;
    }

    size_t qsize() { return this->q.qsize(); }

private:
    std::vector<std::unique_ptr<std::thread>> threads;
    std::thread& get_thread(int i) { return *this->threads[i]; }
    detail::Queue<pmtf::wrap> q;
    detail::Queue<pmtf::wrap> oq;
    unsigned m_numthreads;
    unsigned m_queuedepth;
    std::mutex m_mutex;
    std::condition_variable cv;
    std::atomic<bool> isDone;
    std::atomic<bool> isStop;
    std::atomic<int> nWaiting; // how many threads are waiting
};

} // namespace threadpool
} // namespace schedulers
} // namespace gr