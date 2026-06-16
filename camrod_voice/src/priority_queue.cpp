#include "voice_announcer/priority_queue.hpp"

namespace voice_announcer
{

PriorityQueue::PriorityQueue(Config config)
: config_(config)
{}

bool PriorityQueue::push(const VoiceRequest & req, std::function<void()> interrupt_cb)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Debounce: same non-empty key within debounce window -> reject
  if (!req.key.empty()) {
    auto it = last_seen_.find(req.key);
    if (it != last_seen_.end()) {
      auto elapsed = std::chrono::steady_clock::now() - it->second;
      if (elapsed < config_.debounce_ms) {
        return false;
      }
    }
    last_seen_[req.key] = std::chrono::steady_clock::now();
  }

  // Critical: clear queue and interrupt immediately
  if (req.priority >= 3) {
    queue_ = std::priority_queue<VoiceRequest>{};
    if (interrupt_cb) {
      interrupt_cb();
    }
  } else if (req.interrupt || req.priority >= 2) {
    if (interrupt_cb) {
      interrupt_cb();
    }
  }

  if (queue_.size() >= config_.max_queue_size) {
    return false;
  }

  VoiceRequest r = req;
  r.seq = ++seq_counter_;
  queue_.push(r);
  return true;
}

std::optional<VoiceRequest> PriorityQueue::pop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty()) {
    return std::nullopt;
  }
  auto req = queue_.top();
  queue_.pop();
  return req;
}

std::size_t PriorityQueue::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

bool PriorityQueue::empty() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.empty();
}

void PriorityQueue::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_ = std::priority_queue<VoiceRequest>{};
}

}  // namespace voice_announcer
