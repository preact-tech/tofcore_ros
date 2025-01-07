#include <mutex>
#include <queue>
#include <thread>
// Stole this from here:
// https://www.bit-byter.com/blog/files/blocking-q-cpp.html
template <typename T>
class BlockingQueue
{
private:
    std::mutex mut;
    std::queue<T> private_std_queue;
    std::condition_variable condNotEmpty;
    std::condition_variable condNotFull;
    int count; // Guard with Mutex
    const int MAX{10};

public:
    void put(T new_value)
    {

        std::unique_lock<std::mutex> lk(mut);
        // Condition takes a unique_lock and waits given the false condition
        condNotFull.wait(lk, [this]
                         {
      if (count == MAX) {
        // this used to return false so we cannot push when queue is full,
        // but I would rather just throw out old data than hold up new data
        private_std_queue.pop();
        count--;
        return true;
      } else {
        return true;
      } });
        private_std_queue.push(new_value);
        count++;
        condNotEmpty.notify_one();
    }
    void take(T &value)
    {
        std::unique_lock<std::mutex> lk(mut);
        // Condition takes a unique_lock and waits given the false condition
        condNotEmpty.wait(lk, [this]
                          { return !private_std_queue.empty(); });
        value = private_std_queue.front();
        private_std_queue.pop();
        count--;
        condNotFull.notify_one();
    }
    int length()
    {

        return count;
    }
    double peek_timestamp()
    {
        std::unique_lock<std::mutex> lk(mut);
        // Condition takes a unique_lock and waits given the false condition
        if (private_std_queue.empty())
        {
            lk.unlock();
            return -1;
        }
        T value = private_std_queue.front();
        lk.unlock();

        return static_cast<double>(value.header.stamp.sec) +
               (static_cast<double>(value.header.stamp.nanosec) / 1000000000.0);
    }
};