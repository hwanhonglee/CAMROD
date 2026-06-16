#pragma once

#include <string>
#include <queue>
#include <mutex>
#include <chrono>
#include <unordered_map>
#include <optional>
#include <functional>

namespace voice_announcer
{

struct VoiceRequest
{
  std::string key;
  std::string wav_path;
  uint8_t priority{0};
  bool interrupt{false};
  uint64_t seq{0};  // insertion order — used for FIFO tiebreak within same priority

  bool operator<(const VoiceRequest & other) const
  {
    if (priority != other.priority) {
      return priority < other.priority;  // higher priority value wins
    }
    return seq > other.seq;  // lower seq (earlier insertion) wins — FIFO
  }
};

struct PriorityQueueConfig
{
  std::chrono::milliseconds debounce_ms{3000};
  std::size_t max_queue_size{16};
};

class PriorityQueue
{
public:
  using Config = PriorityQueueConfig;

  explicit PriorityQueue(Config config = Config{});

  // Returns true if the request was accepted into the queue.
  // interrupt=true or priority>=2 triggers the interrupt callback.
  bool push(const VoiceRequest & req, std::function<void()> interrupt_cb = nullptr);

  std::optional<VoiceRequest> pop();

  std::size_t size() const;
  bool empty() const;

  void clear();

private:
  Config config_;
  mutable std::mutex mutex_;
  std::priority_queue<VoiceRequest> queue_;
  uint64_t seq_counter_{0};

  // key -> last accepted time (for debounce)
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_seen_;

  bool isDebounced(const VoiceRequest & req) const;
};

}  // namespace voice_announcer
