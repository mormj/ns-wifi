namespace gr {
namespace schedulers {
namespace threadpool {

namespace detail {
template <typename T>
class Queue
{
public:
    bool push(T const& value)
    {
        std::unique_lock<std::mutex> lock(this->mutex);
        this->q.push(value);
        return true;
    }
    // deletes the retrieved element, do not use for non integral types
    bool pop(T& v)
    {
        std::unique_lock<std::mutex> lock(this->mutex);
        if (this->q.empty())
            return false;
        v = this->q.front();
        this->q.pop();
        return true;
    }
    bool empty()
    {
        std::unique_lock<std::mutex> lock(this->mutex);
        return this->q.empty();
    }
    bool size()
    {
        std::unique_lock<std::mutex> lock(this->mutex);
        return this->q.size();
    }
    size_t qsize()
    {
        std::unique_lock<std::mutex> lock(this->mutex);
        return this->q.size();
    }

private:
    std::queue<T> q;
    std::mutex mutex;
};
} // namespace detail
} // namespace threadpool
} // namespace schedulers
} // namespace gr