// Copyright (c) 2018-2021, Open Source Robotics Foundation, Inc., GAIA Platform, Inc., All rights reserved.  // NOLINT
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <camrod_snapshot/snapshotter.hpp>

#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <csignal>
#include <cassert>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/scope_exit.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace camrod_snapshot
{

using namespace std::chrono_literals;  // NOLINT

using rclcpp::Time;
using avg_msgs::srv::TriggerSnapshot;
using avg_msgs::srv::ConfigureSnapshotTopics;
using avg_msgs::srv::EstimateSnapshot;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::shared_ptr;
using std::string;
using std_srvs::srv::SetBool;

const rclcpp::Duration SnapshotterTopicOptions::NO_DURATION_LIMIT = rclcpp::Duration(-1s);
const int32_t SnapshotterTopicOptions::NO_MEMORY_LIMIT = -1;
const rclcpp::Duration SnapshotterTopicOptions::INHERIT_DURATION_LIMIT = rclcpp::Duration(0s);
const int32_t SnapshotterTopicOptions::INHERIT_MEMORY_LIMIT = 0;
const rclcpp::Duration SnapshotterTopicOptions::NO_MIN_INTERVAL = rclcpp::Duration(0s);
const rclcpp::Duration SnapshotterTopicOptions::INHERIT_MIN_INTERVAL = rclcpp::Duration(-1s);
static constexpr uint32_t MB_TO_B = 1e6;

SnapshotterTopicOptions::SnapshotterTopicOptions(
  rclcpp::Duration duration_limit,
  int32_t memory_limit,
  rclcpp::Duration min_interval)
: duration_limit_(duration_limit), memory_limit_(memory_limit), min_interval_(min_interval)
{
}

SnapshotterOptions::SnapshotterOptions(
  rclcpp::Duration default_duration_limit,
  int32_t default_memory_limit,
  rclcpp::Duration default_min_interval)
: default_duration_limit_(default_duration_limit),
  default_memory_limit_(default_memory_limit),
  default_min_interval_(default_min_interval),
  topics_()
{
}

bool SnapshotterOptions::addTopic(
  const TopicDetails & topic_details,
  rclcpp::Duration duration,
  int32_t memory,
  rclcpp::Duration min_interval)
{
  SnapshotterTopicOptions ops(duration, memory, min_interval);
  std::pair<topics_t::iterator, bool> ret;
  ret = topics_.emplace(topic_details, ops);
  return ret.second;
}

SnapshotterClientOptions::SnapshotterClientOptions()
: action_(SnapshotterClientOptions::TRIGGER_WRITE)
{
}

SnapshotMessage::SnapshotMessage(
  std::shared_ptr<const rclcpp::SerializedMessage> _msg, Time _time)
: msg(_msg), time(_time)
{
}

MessageQueue::MessageQueue(const SnapshotterTopicOptions & options, const rclcpp::Logger & logger)
: options_(options), logger_(logger), size_(0)
{
}

void MessageQueue::setSubscriber(shared_ptr<rclcpp::GenericSubscription> sub)
{
  sub_ = sub;
}

void MessageQueue::clear()
{
  std::lock_guard<std::mutex> l(lock);
  _clear();
}

void MessageQueue::_clear()
{
  queue_.clear();
  size_ = 0;
}

rclcpp::Duration MessageQueue::duration() const
{
  // No duration if 0 or 1 messages
  if (queue_.size() <= 1) {
    return rclcpp::Duration(0s);
  }
  return queue_.back().time - queue_.front().time;
}

bool MessageQueue::preparePush(int32_t size, rclcpp::Time const & time)
{
  // If new message is older than back of queue, time has gone backwards and buffer must be cleared
  if (!queue_.empty() && time < queue_.back().time) {
    RCLCPP_WARN(logger_, "Time has gone backwards. Clearing buffer for this topic.");
    _clear();
  }

  // If throttling is enabled, drop the message when it arrived sooner than
  // min_interval after the last stored message
  if (options_.min_interval_ > SnapshotterTopicOptions::NO_MIN_INTERVAL &&
    !queue_.empty() && time - queue_.back().time < options_.min_interval_)
  {
    return false;
  }

  // The only case where message cannot be addded is if size is greater than limit
  if (options_.memory_limit_ > SnapshotterTopicOptions::NO_MEMORY_LIMIT &&
    size > options_.memory_limit_)
  {
    return false;
  }

  // If memory limit is enforced, remove elements from front of queue until limit
  // would be met once message is added
  if (options_.memory_limit_ > SnapshotterTopicOptions::NO_MEMORY_LIMIT) {
    while (queue_.size() != 0 && size_ + size > options_.memory_limit_) {
      _pop();
    }
  }

  // If duration limit is encforced, remove elements from front of queue until duration limit
  // would be met once message is added
  if (options_.duration_limit_ > SnapshotterTopicOptions::NO_DURATION_LIMIT &&
    queue_.size() != 0)
  {
    rclcpp::Duration dt = time - queue_.front().time;
    while (dt > options_.duration_limit_) {
      _pop();
      if (queue_.empty()) {
        break;
      }
      dt = time - queue_.front().time;
    }
  }
  return true;
}
void MessageQueue::push(SnapshotMessage const & _out)
{
  auto ret = lock.try_lock();
  if (!ret) {
    RCLCPP_ERROR(logger_, "Failed to lock. Time %f", _out.time.seconds());
    return;
  }
  _push(_out);
  if (ret) {
    lock.unlock();
  }
}

SnapshotMessage MessageQueue::pop()
{
  std::lock_guard<std::mutex> l(lock);
  return _pop();
}

int64_t MessageQueue::getMessageSize(SnapshotMessage const & snapshot_msg) const
{
  return snapshot_msg.msg->size() + sizeof(SnapshotMessage);
}

void MessageQueue::_push(SnapshotMessage const & _out)
{
  int32_t size = _out.msg->size();
  // If message cannot be added without violating limits, it must be dropped
  if (!preparePush(size, _out.time)) {
    return;
  }
  queue_.push_back(_out);
  // Add size of new message to running count to maintain correctness
  size_ += getMessageSize(_out);
}

SnapshotMessage MessageQueue::_pop()
{
  SnapshotMessage tmp = queue_.front();
  queue_.pop_front();
  //  Remove size of popped message to maintain correctness of size_
  size_ -= getMessageSize(tmp);
  return tmp;
}

MessageQueue::range_t MessageQueue::rangeFromTimes(Time const & start, Time const & stop)
{
  range_t::first_type begin = queue_.begin();
  range_t::second_type end = queue_.end();

  // Increment / Decrement iterators until time contraints are met
  if (start.seconds() != 0.0 || start.nanoseconds() != 0) {
    while (begin != end && (*begin).time < start) {
      ++begin;
    }
  }
  if (stop.seconds() != 0.0 || stop.nanoseconds() != 0) {
    while (end != begin && (*(end - 1)).time > stop) {
      --end;
    }
  }
  return range_t(begin, end);
}

const int Snapshotter::QUEUE_SIZE = 10;

Snapshotter::Snapshotter(const rclcpp::NodeOptions & options)
: rclcpp::Node("snapshotter", options),
  recording_(true),
  writing_(false)
{
  parseOptionsFromParams();

  // Create the queue for each topic and set up the subscriber to add to it on new messages
  for (auto & pair : options_.topics_) {
    string topic{pair.first.name}, type{pair.first.type};
    fixTopicOptions(pair.second);
    shared_ptr<MessageQueue> queue;
    queue.reset(new MessageQueue(pair.second, get_logger()));

    TopicDetails details{};
    details.name = topic;
    details.type = type;
    std::pair<buffers_t::iterator, bool> res =
      buffers_.emplace(details, queue);
    assert(res.second);

    subscribe(details, queue);
  }

  // Now that subscriptions are setup, setup service servers for writing and pausing
  trigger_snapshot_server_ = create_service<TriggerSnapshot>(
    "trigger_snapshot", std::bind(&Snapshotter::triggerSnapshotCb, this, _1, _2, _3));
  estimate_snapshot_server_ = create_service<EstimateSnapshot>(
    "estimate_snapshot", std::bind(&Snapshotter::estimateSnapshotCb, this, _1, _2, _3));
  configure_topics_server_ = create_service<ConfigureSnapshotTopics>(
    "configure_snapshot_topics",
    std::bind(&Snapshotter::configureTopicsCb, this, _1, _2, _3));
  enable_server_ = create_service<SetBool>(
    "enable_snapshot", std::bind(&Snapshotter::enableCb, this, _1, _2, _3));

  // Start timer to poll for topics
  if (options_.all_topics_) {
    poll_topic_timer_ =
      create_wall_timer(
      std::chrono::duration(1s),
      std::bind(&Snapshotter::pollTopics, this));
  }

  // Subscribe last: a rule may fire as soon as its first message arrives, and
  // the write path must already be constructed when it does.
  parseAutoTriggerParams();
  startAutoTrigger();

  parseOffloadParams();
  startOffload();
}

Snapshotter::~Snapshotter()
{
  // Join the capture worker before the buffers it writes from are destroyed.
  stopAutoTrigger();
  stopOffload();

  for (auto & buffer : buffers_) {
    buffer.second->sub_.reset();
  }
}

void Snapshotter::parseOptionsFromParams()
{
  std::vector<std::string> topics{};

  const int64_t split_duration_s =
    declare_parameter<int64_t>("bagfile_split_duration_s", 60);
  if (split_duration_s <= 0) {
    throw std::invalid_argument("bagfile_split_duration_s must be greater than zero");
  }
  bagfile_split_duration_s_ = static_cast<uint64_t>(split_duration_s);

  try {
    options_.default_duration_limit_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("default_duration_limit", -1.0));
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "default_duration_limit is of incorrect type.");
    throw ex;
  }

  try {
    options_.default_memory_limit_ =
      declare_parameter<double>("default_memory_limit", -1.0);
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "default_memory_limit is of incorrect type.");
    throw ex;
  }

  // Convert memory limit in MB to B
  if (options_.default_memory_limit_ != -1.0) {
    options_.default_memory_limit_ *= MB_TO_B;
  }

  try {
    options_.default_min_interval_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("default_min_interval", 0.0));
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "default_min_interval is of incorrect type.");
    throw ex;
  }

  try {
    topics = declare_parameter<std::vector<std::string>>(
      "topics", std::vector<std::string>{});
  } catch (const rclcpp::ParameterTypeException & ex) {
    if (std::string{ex.what()}.find("not set") == std::string::npos) {
      RCLCPP_ERROR(get_logger(), "topics must be an array of strings.");
      throw ex;
    }
  }

  if (topics.size() > 0) {
    options_.all_topics_ = false;

    for (const auto & topic : topics) {
      std::string prefix = "topic_details." + topic;
      std::string topic_type{};
      SnapshotterTopicOptions opts{};

      try {
        topic_type = declare_parameter<std::string>(prefix + ".type");
      } catch (const rclcpp::ParameterTypeException & ex) {
        if (std::string{ex.what()}.find("not set") == std::string::npos) {
          RCLCPP_ERROR(get_logger(), "Topic type must be a string.");
        } else {
          RCLCPP_ERROR(get_logger(), "Topic %s is missing a type.", topic.c_str());
        }

        throw ex;
      }

      // Optional per-topic settings use defaults mapping to the INHERIT sentinels
      // (duration 0.0 = INHERIT_DURATION_LIMIT, memory 0.0 = INHERIT_MEMORY_LIMIT,
      // interval -1.0 = INHERIT_MIN_INTERVAL) so an omitted key falls back to the
      // node-level default instead of aborting on Humble's statically typed parameters.
      try {
        opts.duration_limit_ = rclcpp::Duration::from_seconds(
          declare_parameter<double>(prefix + ".duration", 0.0)
        );
      } catch (const rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(
          get_logger(), "Duration limit for topic %s must be a double.", topic.c_str());
        throw ex;
      }

      try {
        opts.memory_limit_ = declare_parameter<double>(prefix + ".memory", 0.0);
        // Convert memory limit in MB to B, matching default_memory_limit
        if (opts.memory_limit_ != SnapshotterTopicOptions::INHERIT_MEMORY_LIMIT &&
          opts.memory_limit_ != SnapshotterTopicOptions::NO_MEMORY_LIMIT)
        {
          opts.memory_limit_ *= MB_TO_B;
        }
      } catch (const rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(
          get_logger(), "Memory limit for topic %s is of the wrong type.", topic.c_str());
        throw ex;
      }

      try {
        opts.min_interval_ = rclcpp::Duration::from_seconds(
          declare_parameter<double>(prefix + ".interval", -1.0)
        );
      } catch (const rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(
          get_logger(), "Interval for topic %s must be a double.", topic.c_str());
        throw ex;
      }

      TopicDetails dets{};
      dets.name = topic;
      dets.type = topic_type;

      options_.topics_.insert(
        SnapshotterOptions::topics_t::value_type(dets, opts));
    }
  } else {
    options_.all_topics_ = true;
    RCLCPP_INFO(get_logger(), "No topics list provided. Logging all topics.");
    RCLCPP_WARN(get_logger(), "Logging all topics is very memory-intensive.");
  }
}

void Snapshotter::fixTopicOptions(SnapshotterTopicOptions & options)
{
  if (options.duration_limit_ == SnapshotterTopicOptions::INHERIT_DURATION_LIMIT) {
    options.duration_limit_ = options_.default_duration_limit_;
  }
  if (options.memory_limit_ == SnapshotterTopicOptions::INHERIT_MEMORY_LIMIT) {
    options.memory_limit_ = options_.default_memory_limit_;
  }
  if (options.min_interval_ == SnapshotterTopicOptions::INHERIT_MIN_INTERVAL) {
    options.min_interval_ = options_.default_min_interval_;
  }
}

bool Snapshotter::postfixFilename(string & file)
{
  size_t ind = file.rfind(".bag");
  // If requested ends in .bag, this is literal name do not append date
  if (ind != string::npos && ind == file.size() - 4) {
    return true;
  }
  // Otherwise treat as prefix and append datetime and extension
  file += timeAsStr() + ".bag";
  return true;
}

string Snapshotter::timeAsStr()
{
  std::stringstream msg;
  const auto now = std::chrono::system_clock::now();
  const auto now_in_t = std::chrono::system_clock::to_time_t(now);
  msg << std::put_time(std::localtime(&now_in_t), "%Y-%m-%d-%H-%M-%S");
  return msg.str();
}

void Snapshotter::topicCb(
  std::shared_ptr<const rclcpp::SerializedMessage> msg,
  std::shared_ptr<MessageQueue> queue)
{
  // If recording is paused (or writing), exit
  {
    std::shared_lock<std::shared_mutex> lock(state_lock_);
    if (!recording_) {
      return;
    }
  }

  // Pack message and metadata into SnapshotMessage holder
  SnapshotMessage out(msg, now());
  queue->push(out);
}

void Snapshotter::subscribe(
  const TopicDetails & topic_details,
  std::shared_ptr<MessageQueue> queue)
{
  RCLCPP_INFO(get_logger(), "Subscribing to %s", topic_details.name.c_str());

  auto opts = rclcpp::SubscriptionOptions{};
  opts.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  opts.topic_stats_options.publish_topic = topic_details.name + "/statistics";

  // BEST_EFFORT subscribers are compatible with both sensor-data and reliable
  // publishers. tf_static is the durability exception: it must receive the
  // publisher's retained transform even when snapshotter starts later.
  auto qos = rclcpp::QoS{10}.best_effort().durability_volatile();
  if (topic_details.name == "/tf_static") {
    qos = rclcpp::QoS{100}.reliable().transient_local();
  }

  auto sub = create_generic_subscription(
    topic_details.name,
    topic_details.type,
    qos,
    std::bind(&Snapshotter::topicCb, this, _1, queue),
    opts
  );

  queue->setSubscriber(sub);
}

Snapshotter::selected_buffers_t Snapshotter::selectBuffers(
  const std::vector<DetailsMsg> & requested_topics)
{
  selected_buffers_t selected;
  std::lock_guard<std::mutex> lock(buffers_lock_);
  if (!requested_topics.empty() && !requested_topics.front().name.empty() &&
    !requested_topics.front().type.empty())
  {
    selected.reserve(requested_topics.size());
    for (const auto & topic : requested_topics) {
      TopicDetails details{topic.name, topic.type};
      auto found = buffers_.find(details);
      if (found == buffers_.end()) {
        RCLCPP_WARN(
          get_logger(), "Requested topic %s is not subscribed, skipping.", topic.name.c_str());
        continue;
      }
      selected.emplace_back(details, found->second);
    }
  } else {
    selected.reserve(buffers_.size());
    for (const buffers_t::value_type & pair : buffers_) {
      selected.emplace_back(pair.first, pair.second);
    }
  }
  return selected;
}

Snapshotter::SnapshotEstimateResult Snapshotter::estimateBuffers(
  const selected_buffers_t & selected,
  const rclcpp::Time & start,
  const rclcpp::Time & stop,
  uint64_t max_bytes)
{
  struct TimedSize
  {
    int64_t time_ns;
    uint64_t size;
  };

  SnapshotEstimateResult result;
  std::vector<TimedSize> messages;
  int64_t oldest_ns{0};
  for (const auto & pair : selected) {
    auto & queue = *(pair.second);
    std::lock_guard<std::mutex> lock(queue.lock);
    const MessageQueue::range_t range = queue.rangeFromTimes(start, stop);
    for (auto message = range.first; message != range.second; ++message) {
      const uint64_t size = static_cast<uint64_t>(message->msg->size());
      const int64_t time_ns = message->time.nanoseconds();
      messages.push_back({time_ns, size});
      result.requested_bytes += size;
      if (oldest_ns == 0 || time_ns < oldest_ns) {
        oldest_ns = time_ns;
      }
      if (time_ns > result.newest_ns) {
        result.newest_ns = time_ns;
      }
    }
  }

  if (messages.empty()) {
    result.message = TriggerSnapshot::Response::NO_DATA_MESSAGE;
    return result;
  }

  result.success = true;
  result.selected_bytes = result.requested_bytes;
  result.message_count = messages.size();
  result.actual_start_ns = oldest_ns;
  if (max_bytes == 0 || result.requested_bytes <= max_bytes) {
    result.message = "Requested snapshot window fits the byte budget.";
    return result;
  }

  std::sort(
    messages.begin(), messages.end(),
    [](const TimedSize & left, const TimedSize & right) {
      return left.time_ns > right.time_ns;
    });

  uint64_t selected_bytes{0};
  uint64_t selected_messages{0};
  int64_t cutoff_ns{0};
  for (size_t index = 0; index < messages.size();) {
    const int64_t group_time_ns = messages[index].time_ns;
    uint64_t group_bytes{0};
    uint64_t group_messages{0};
    while (index < messages.size() && messages[index].time_ns == group_time_ns) {
      group_bytes += messages[index].size;
      ++group_messages;
      ++index;
    }
    if (selected_bytes + group_bytes > max_bytes) {
      break;
    }
    selected_bytes += group_bytes;
    selected_messages += group_messages;
    cutoff_ns = group_time_ns;
  }

  if (selected_messages == 0) {
    result.success = false;
    result.selected_bytes = 0;
    result.message_count = 0;
    result.message = "The byte budget is too small for the newest buffered message.";
    return result;
  }

  result.selected_bytes = selected_bytes;
  result.message_count = selected_messages;
  result.actual_start_ns = cutoff_ns;
  result.truncated = true;
  result.message = "Snapshot window was reduced to the newest data that fits the byte budget.";
  return result;
}

bool Snapshotter::hasMinimumDiskSpace(
  const std::string & filename, uint64_t minimum_free_bytes) const
{
  if (minimum_free_bytes == 0) {
    return true;
  }
  std::error_code error;
  auto probe = std::filesystem::path(filename).parent_path();
  if (probe.empty()) {
    probe = std::filesystem::current_path(error);
  }
  const auto space = std::filesystem::space(probe, error);
  return !error && space.available > minimum_free_bytes;
}

void Snapshotter::estimateSnapshotCb(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const EstimateSnapshot::Request::SharedPtr req,
  EstimateSnapshot::Response::SharedPtr res)
{
  (void)request_header;
  const auto selected = selectBuffers(req->topics);
  const auto estimate = estimateBuffers(
    selected, rclcpp::Time(req->start_time), rclcpp::Time(req->stop_time), req->max_bytes);
  res->success = estimate.success;
  res->requested_bytes = estimate.requested_bytes;
  res->selected_bytes = estimate.selected_bytes;
  res->message_count = estimate.message_count;
  res->actual_start_time = static_cast<builtin_interfaces::msg::Time>(
    rclcpp::Time(estimate.actual_start_ns));
  res->newest_time = static_cast<builtin_interfaces::msg::Time>(
    rclcpp::Time(estimate.newest_ns));
  res->truncated = estimate.truncated;
  res->message = estimate.message;
}

bool Snapshotter::writeTopic(
  rosbag2_cpp::Writer & bag_writer,
  MessageQueue & message_queue,
  const TopicDetails & topic_details,
  const TriggerSnapshot::Request::SharedPtr & req,
  const TriggerSnapshot::Response::SharedPtr & res,
  uint64_t & messages_written,
  uint64_t & bytes_written,
  uint64_t & bytes_since_space_check)
{
  // acquire lock for this queue
  std::lock_guard l(message_queue.lock);

  MessageQueue::range_t range = message_queue.rangeFromTimes(req->start_time, req->stop_time);

  rosbag2_storage::TopicMetadata tm;
  tm.name = topic_details.name;
  tm.type = topic_details.type;
  tm.serialization_format = "cdr";

  bag_writer.create_topic(tm);

  for (auto msg_it = range.first; msg_it != range.second; ++msg_it) {
    const uint64_t message_size = static_cast<uint64_t>(msg_it->msg->size());
    if (req->minimum_free_bytes > 0 &&
      (bytes_since_space_check >= 16U * 1000U * 1000U || message_size >= 16U * 1000U * 1000U))
    {
      if (!hasMinimumDiskSpace(req->filename, req->minimum_free_bytes + message_size)) {
        res->message = "Snapshot stopped before the filesystem safety reserve was exhausted.";
        return false;
      }
      bytes_since_space_check = 0;
    }
    // Create BAG message
    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    auto ret = rcutils_system_time_now(&bag_message->time_stamp);
    if (ret != RCL_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Failed to assign time to rosbag message.");
      return false;
    }

    bag_message->topic_name = tm.name;
    bag_message->time_stamp = msg_it->time.nanoseconds();
    bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>(
      msg_it->msg->get_rcl_serialized_message()
    );

    bag_writer.write(bag_message);
    ++messages_written;
    bytes_written += message_size;
    bytes_since_space_check += message_size;
  }

  return true;
}

void Snapshotter::triggerSnapshotCb(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const TriggerSnapshot::Request::SharedPtr req,
  TriggerSnapshot::Response::SharedPtr res)
{
  (void)request_header;
  writeSnapshot(req, res, "service");
}

void Snapshotter::writeSnapshot(
  const TriggerSnapshot::Request::SharedPtr & req,
  const TriggerSnapshot::Response::SharedPtr & res,
  const std::string & origin)
{
  if (req->filename.empty() || !postfixFilename(req->filename)) {
    res->success = false;
    res->message = "Invalid filename";
    return;
  }

  // Store if we were recording prior to write to restore this state after write
  bool recording_prior{true};
  // HH_260922 - Only a write that actually opened the bag leaves a gap worth
  // clearing for. Every pre-flight rejection below - no data, a byte budget
  // that fits nothing, an unavailable filesystem reserve - happens in the time
  // it takes to stat the buffer and the disk, so the buffer is still coherent.
  // Discarding it there would cost the operator the very history the snapshot
  // was meant to preserve, and on a full disk that history is irreplaceable.
  bool bag_opened{false};

  {
    std::unique_lock<std::shared_mutex> write_lock(state_lock_);
    if (writing_) {
      res->success = false;
      res->message = "Already writing";
      return;
    }
    recording_prior = recording_;
    if (recording_prior) {
      pause();
    }
    writing_ = true;
  }

  // Ensure that state is updated when function exits, regardlesss of branch path / exception events
  RCPPUTILS_SCOPE_EXIT(
    // Clear buffers beacuase time gaps (skipped messages) may have occured while paused
    std::unique_lock<std::shared_mutex> write_lock(state_lock_);
    // Turn off writing flag and return recording to its state before writing
    writing_ = false;
    if (recording_prior) {
      this->resume(bag_opened);
    }
  );

  // Copy shared queue handles so runtime topic changes cannot invalidate map
  // iterators while the potentially slow bag write is in progress.
  const auto selected = selectBuffers(req->topics);
  const auto estimate = estimateBuffers(
    selected, rclcpp::Time(req->start_time), rclcpp::Time(req->stop_time), req->max_bytes);
  res->requested_bytes = estimate.requested_bytes;
  res->selected_bytes = estimate.selected_bytes;
  res->message_count = estimate.message_count;
  res->actual_start_time = static_cast<builtin_interfaces::msg::Time>(
    rclcpp::Time(estimate.actual_start_ns));
  res->truncated = estimate.truncated;
  if (!estimate.success) {
    res->success = false;
    res->message = estimate.message;
    return;
  }
  if (estimate.truncated) {
    req->start_time = res->actual_start_time;
  }

  if (!hasMinimumDiskSpace(req->filename, req->minimum_free_bytes)) {
    res->success = false;
    res->message = "Snapshot rejected because the filesystem safety reserve is unavailable.";
    return;
  }

  rosbag2_cpp::Writer bag_writer{};
  try {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = req->filename;
    storage_options.max_bagfile_duration = bagfile_split_duration_s_;
    bag_writer.open(storage_options);
    // From here on the pause has lasted as long as a bag write, and messages
    // were dropped meanwhile, so the buffer is no longer continuous.
    bag_opened = true;
  } catch (const std::exception & ex) {
    res->success = false;
    res->message = "Unable to open file for writing.";
    return;
  }

  uint64_t messages_written{0};
  uint64_t bytes_written{0};
  uint64_t bytes_since_space_check{0};
  size_t topics_written{0};
  for (const auto & pair : selected) {
    const uint64_t before = messages_written;
    if (!writeTopic(
        bag_writer, *(pair.second), pair.first, req, res, messages_written,
        bytes_written, bytes_since_space_check))
    {
      res->success = false;
      if (res->message.empty()) {
        res->message = "Failed to write topic " + pair.first.name + " to bag file.";
      }
      return;
    }
    if (messages_written > before) {
      ++topics_written;
    }
  }

  if (messages_written == 0) {
    res->success = false;
    res->message = res->NO_DATA_MESSAGE;
    return;
  }

  res->success = true;
  res->selected_bytes = bytes_written;
  res->message_count = messages_written;
  res->message = "Saved " + std::to_string(messages_written) + " messages from " +
    std::to_string(topics_written) + " topics to " + req->filename +
    (estimate.truncated ? " (latest data only; fitted to disk budget)." : ".");

  // Queue rather than transfer here: this runs on the service thread for an
  // operator-triggered snapshot, and the caller is waiting on the response.
  enqueueOffload(req->filename, origin);
}

void Snapshotter::configureTopicsCb(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const ConfigureSnapshotTopics::Request::SharedPtr req,
  ConfigureSnapshotTopics::Response::SharedPtr res)
{
  (void)request_header;

  // Retain the read lock through registry mutation. A snapshot trigger must
  // either start before this callback (and reject the mutation) or afterwards.
  std::shared_lock<std::shared_mutex> state_read_lock(state_lock_);
  res->recording = recording_;
  res->writing = writing_;
  if (writing_ && (!req->add_topics.empty() || !req->remove_topics.empty())) {
    res->success = false;
    res->message = "Cannot change topics while a snapshot is being written.";
  }

  std::vector<std::string> rejected;
  size_t added{0};
  size_t removed{0};

  if (!res->writing || (req->add_topics.empty() && req->remove_topics.empty())) {
    std::lock_guard<std::mutex> lock(buffers_lock_);

    // Base topics come from the reviewed YAML and cannot be removed at runtime.
    for (std::string name : req->remove_topics) {
      if (!name.empty() && name.front() != '/') {
        name.insert(name.begin(), '/');
      }
      auto dynamic = dynamic_topics_.find(name);
      if (dynamic == dynamic_topics_.end()) {
        rejected.push_back(name);
        continue;
      }
      buffers_.erase(dynamic->second);
      dynamic_topics_.erase(dynamic);
      ++removed;
    }

    const auto graph_topics = get_topic_names_and_types();
    for (std::string name : req->add_topics) {
      if (!name.empty() && name.front() != '/') {
        name.insert(name.begin(), '/');
      }
      if (name.empty() || name == "/") {
        rejected.push_back(name);
        continue;
      }

      bool already_active{false};
      for (const auto & pair : buffers_) {
        if (pair.first.name == name) {
          already_active = true;
          break;
        }
      }
      if (already_active) {
        continue;
      }

      const auto discovered = graph_topics.find(name);
      if (discovered == graph_topics.end() || discovered->second.size() != 1) {
        rejected.push_back(name);
        continue;
      }

      TopicDetails details{name, discovered->second.front()};
      SnapshotterTopicOptions topic_options;
      fixTopicOptions(topic_options);
      // Apply conservative runtime defaults to payload-heavy topic types.
      const bool is_uncompressed_payload =
        details.type == "sensor_msgs/msg/Image" ||
        details.type == "sensor_msgs/msg/PointCloud2";
      const bool is_grid =
        details.type == "nav_msgs/msg/OccupancyGrid" ||
        details.type == "nav2_msgs/msg/Costmap" ||
        details.type == "avg_msgs/msg/AvgOccupancyGrid";
      if (details.type == "sensor_msgs/msg/CompressedImage") {
        topic_options.duration_limit_ = rclcpp::Duration::from_seconds(120.0);
        topic_options.min_interval_ = rclcpp::Duration::from_seconds(0.5);
      } else if (is_uncompressed_payload) {
        topic_options.duration_limit_ = rclcpp::Duration::from_seconds(60.0);
        topic_options.min_interval_ = rclcpp::Duration::from_seconds(1.0);
      } else if (is_grid) {
        topic_options.min_interval_ = rclcpp::Duration::from_seconds(0.5);
      }

      auto queue = std::make_shared<MessageQueue>(topic_options, get_logger());
      subscribe(details, queue);
      buffers_.emplace(details, queue);
      dynamic_topics_.emplace(name, details);
      ++added;
    }

    for (const auto & pair : buffers_) {
      res->active_topics.push_back(pair.first.asMessage());
    }
    for (const auto & pair : dynamic_topics_) {
      res->dynamic_topics.push_back(pair.second.asMessage());
    }
  }

  res->rejected_topics = rejected;
  if (!res->writing || (req->add_topics.empty() && req->remove_topics.empty())) {
    res->success = rejected.empty();
    res->message = "Added " + std::to_string(added) + ", removed " +
      std::to_string(removed) + ", active " + std::to_string(res->active_topics.size()) +
      (rejected.empty() ? "." : "; some topics were rejected.");
  }
}

void Snapshotter::clear()
{
  std::vector<std::shared_ptr<MessageQueue>> queues;
  {
    std::lock_guard<std::mutex> lock(buffers_lock_);
    queues.reserve(buffers_.size());
    for (const buffers_t::value_type & pair : buffers_) {
      queues.push_back(pair.second);
    }
  }
  for (const auto & queue : queues) {
    queue->clear();
  }
}

void Snapshotter::pause()
{
  RCLCPP_INFO(get_logger(), "Buffering paused");
  recording_ = false;
}

void Snapshotter::resume(const bool clear_buffers)
{
  if (clear_buffers) {
    clear();
  }
  recording_ = true;
  const char * const note = clear_buffers ?
    "Buffering resumed and old data cleared." :
    "Buffering resumed with the existing buffer intact.";
  RCLCPP_INFO(get_logger(), "%s", note);
}

void Snapshotter::enableCb(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const SetBool::Request::SharedPtr req,
  SetBool::Response::SharedPtr res)
{
  (void)request_header;

  {
    std::shared_lock<std::shared_mutex> read_lock(state_lock_);
    // Cannot enable while writing
    if (req->data && writing_) {
      res->success = false;
      res->message = "cannot enable recording while writing.";
      return;
    }
  }

  // Obtain write lock and update state if requested state is different from current
  if (req->data && !recording_) {
    std::unique_lock<std::shared_mutex> write_lock(state_lock_);
    resume();
  } else if (!req->data && recording_) {
    std::unique_lock<std::shared_mutex> write_lock(state_lock_);
    pause();
  }

  res->success = true;
}

void Snapshotter::pollTopics()
{
  const auto topic_names_and_types = get_topic_names_and_types();

  for (const auto & name_type : topic_names_and_types) {
    if (name_type.second.size() < 1) {
      RCLCPP_ERROR(get_logger(), "Subscribed topic has no associated type.");
      return;
    }

    if (name_type.second.size() > 1) {
      RCLCPP_ERROR(get_logger(), "Subscribed topic has more than one associated type.");
      return;
    }

    TopicDetails details{};
    details.name = name_type.first;
    details.type = name_type.second[0];

    if (options_.addTopic(details)) {
      SnapshotterTopicOptions topic_options;
      fixTopicOptions(topic_options);
      auto queue = std::make_shared<MessageQueue>(topic_options, get_logger());

      // HH_260921 - An automatic capture writes from its own thread, so the
      // registry may no longer be mutated on the executor alone.
      {
        std::lock_guard<std::mutex> lock(buffers_lock_);
        std::pair<buffers_t::iterator,
          bool> res = buffers_.emplace(details, queue);
        assert(res.second);
      }
      subscribe(details, queue);
    }
  }
}

double Snapshotter::steadySeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void Snapshotter::parseAutoTriggerParams()
{
  auto_trigger_enabled_ = declare_parameter<bool>("auto_trigger.enabled", false);
  auto_trigger_directory_ =
    declare_parameter<std::string>("auto_trigger.output_directory", "");
  auto_trigger_prefix_ =
    declare_parameter<std::string>("auto_trigger.filename_prefix", "autosnapshot");
  auto_trigger_cooldown_s_ = declare_parameter<double>("auto_trigger.cooldown_s", 0.0);
  auto_trigger_startup_grace_s_ =
    declare_parameter<double>("auto_trigger.startup_grace_s", 60.0);
  auto_trigger_lookback_s_ = declare_parameter<double>("auto_trigger.lookback_s", 0.0);
  auto_trigger_minimum_free_mb_ = static_cast<uint64_t>(
    std::max<int64_t>(0, declare_parameter<int64_t>("auto_trigger.minimum_free_space_mb", 5120)));
  auto_trigger_minimum_free_ratio_ =
    declare_parameter<double>("auto_trigger.minimum_free_space_ratio", 0.10);
  auto_trigger_size_safety_factor_ =
    declare_parameter<double>("auto_trigger.size_safety_factor", 1.30);

  // Writing a snapshot pauses recording and clears every buffer afterwards.
  // Without a cooldown the next capture would write the history the previous
  // one just discarded, which is exactly the storm a fault cascade produces.
  if (auto_trigger_cooldown_s_ <= 0.0) {
    const double buffer_s = options_.default_duration_limit_.seconds();
    auto_trigger_cooldown_s_ = buffer_s > 0.0 ? buffer_s : 300.0;
  }
  if (auto_trigger_startup_grace_s_ < 0.0) {
    throw std::invalid_argument("auto_trigger.startup_grace_s must not be negative");
  }
  if (auto_trigger_lookback_s_ < 0.0) {
    throw std::invalid_argument("auto_trigger.lookback_s must not be negative");
  }
  if (auto_trigger_size_safety_factor_ < 1.0) {
    throw std::invalid_argument("auto_trigger.size_safety_factor must be at least 1.0");
  }
  if (auto_trigger_minimum_free_ratio_ < 0.0 || auto_trigger_minimum_free_ratio_ >= 1.0) {
    throw std::invalid_argument("auto_trigger.minimum_free_space_ratio must be in [0.0, 1.0)");
  }

  const auto rule_names = declare_parameter<std::vector<std::string>>(
    "auto_trigger.rules", std::vector<std::string>{});

  std::set<std::string> seen_names;
  for (const auto & name : rule_names) {
    // The rule name becomes part of the bag filename, so keep it to a
    // portable, shell-safe alphabet instead of sanitizing it later.
    const bool portable_name = !name.empty() &&
      std::all_of(
      name.begin(), name.end(), [](const unsigned char character) {
        return std::isalnum(character) != 0 || character == '_' || character == '-';
      });
    if (!portable_name) {
      throw std::invalid_argument(
              "auto_trigger.rules entries must be non-empty and use only "
              "letters, digits, '_' or '-'");
    }
    if (!seen_names.insert(name).second) {
      throw std::invalid_argument("auto_trigger.rules contains duplicate rule '" + name + "'");
    }

    auto rule = std::make_shared<AutoTriggerRule>();
    rule->name = name;

    const std::string prefix = "auto_trigger.rule." + name + ".";
    rule->topic = declare_parameter<std::string>(prefix + "topic", "");
    const auto kind = declare_parameter<std::string>(prefix + "kind", "module_state");
    const auto states = declare_parameter<std::vector<std::string>>(
      prefix + "operating_states", std::vector<std::string>{});
    const auto modules = declare_parameter<std::vector<std::string>>(
      prefix + "modules", std::vector<std::string>{});
    rule->min_level = static_cast<int>(declare_parameter<int64_t>(prefix + "min_level", -1));
    rule->on_system_not_ok = declare_parameter<bool>(prefix + "on_system_not_ok", false);
    rule->hold_s = declare_parameter<double>(prefix + "hold_s", 0.0);
    rule->require_healthy_first =
      declare_parameter<bool>(prefix + "require_healthy_first", true);
    rule->require_healthy_s = declare_parameter<double>(prefix + "require_healthy_s", 0.0);
    rule->min_occurrences =
      static_cast<int>(declare_parameter<int64_t>(prefix + "min_occurrences", 1));
    rule->scope_topic = declare_parameter<std::string>(prefix + "scope_topic", "");
    const auto scope_kind =
      declare_parameter<std::string>(prefix + "scope_kind", "service_state");
    const auto scope_active_states = declare_parameter<std::vector<std::string>>(
      prefix + "scope_active_states", std::vector<std::string>{});
    const auto scope_reset_states = declare_parameter<std::vector<std::string>>(
      prefix + "scope_reset_states", std::vector<std::string>{});

    if (rule->topic.empty() || rule->topic.front() != '/') {
      throw std::invalid_argument(
              "auto_trigger rule '" + name + "' needs an absolute topic name");
    }
    if (kind == "module_state") {
      rule->kind = AutoTriggerRule::Kind::kModuleState;
    } else if (kind == "system_status") {
      rule->kind = AutoTriggerRule::Kind::kSystemStatus;
    } else if (kind == "service_state") {
      rule->kind = AutoTriggerRule::Kind::kServiceState;
    } else {
      throw std::invalid_argument(
              "auto_trigger rule '" + name +
              "' kind must be module_state, system_status or service_state");
    }
    if (scope_kind == "module_state") {
      rule->scope_kind = AutoTriggerRule::Kind::kModuleState;
    } else if (scope_kind == "system_status") {
      rule->scope_kind = AutoTriggerRule::Kind::kSystemStatus;
    } else if (scope_kind == "service_state") {
      rule->scope_kind = AutoTriggerRule::Kind::kServiceState;
    } else {
      throw std::invalid_argument(
              "auto_trigger rule '" + name +
              "' scope_kind must be module_state, system_status or service_state");
    }
    rule->scope_active_states.insert(scope_active_states.begin(), scope_active_states.end());
    rule->scope_reset_states.insert(scope_reset_states.begin(), scope_reset_states.end());
    // A rule that only counts inside named states must wait for the scope
    // topic to say it is in one; a rule without that list is live from
    // the start.
    rule->scope_active = rule->scope_active_states.empty();
    if (rule->hold_s < 0.0) {
      throw std::invalid_argument("auto_trigger rule '" + name + "' hold_s must not be negative");
    }
    if (rule->require_healthy_s < 0.0) {
      throw std::invalid_argument(
              "auto_trigger rule '" + name + "' require_healthy_s must not be negative");
    }
    // A rule that must see health first starts disarmed. Nothing it receives
    // before that first healthy report can write a bag.
    rule->armed = !rule->require_healthy_first;
    if (rule->min_level > 255) {
      throw std::invalid_argument("auto_trigger rule '" + name + "' min_level must fit a uint8");
    }
    rule->operating_states.insert(states.begin(), states.end());
    rule->module_names.insert(modules.begin(), modules.end());

    if (rule->min_occurrences < 1) {
      throw std::invalid_argument(
              "auto_trigger rule '" + name + "' min_occurrences must be at least 1");
    }
    const bool has_scope_states =
      !rule->scope_active_states.empty() || !rule->scope_reset_states.empty();
    if (rule->min_occurrences > 1 && rule->scope_reset_states.empty()) {
      // An unscoped count would accumulate across unrelated missions until it
      // eventually tripped on events that were never related.
      throw std::invalid_argument(
              "auto_trigger rule '" + name +
              "' sets min_occurrences > 1 and so needs scope_reset_states to bound the count");
    }
    if (rule->scope_topic.empty() == has_scope_states) {
      throw std::invalid_argument(
              "auto_trigger rule '" + name +
              "' needs scope_topic together with scope_active_states and/or "
              "scope_reset_states");
    }
    if (!rule->scope_topic.empty() && rule->scope_topic.front() != '/') {
      throw std::invalid_argument(
              "auto_trigger rule '" + name + "' scope_topic must be an absolute topic name");
    }
    if (rule->kind == AutoTriggerRule::Kind::kServiceState &&
      (rule->min_level >= 0 || rule->on_system_not_ok))
    {
      throw std::invalid_argument(
              "auto_trigger rule '" + name +
              "' uses kind: service_state, which carries no severity; match operating_states");
    }
    if (rule->operating_states.empty() && rule->min_level < 0 && !rule->on_system_not_ok) {
      throw std::invalid_argument(
              "auto_trigger rule '" + name + "' has no condition: set operating_states, "
              "min_level or on_system_not_ok");
    }
    if (rule->kind == AutoTriggerRule::Kind::kModuleState &&
      (rule->on_system_not_ok || !rule->module_names.empty()))
    {
      throw std::invalid_argument(
              "auto_trigger rule '" + name + "' sets on_system_not_ok or modules, which apply "
              "only to kind: system_status");
    }

    auto_trigger_rules_.push_back(rule);
  }

  if (!auto_trigger_enabled_) {
    return;
  }
  if (auto_trigger_rules_.empty()) {
    throw std::invalid_argument(
            "auto_trigger.enabled requires at least one entry in auto_trigger.rules");
  }
  if (auto_trigger_directory_.empty()) {
    throw std::invalid_argument("auto_trigger.enabled requires auto_trigger.output_directory");
  }
}

void Snapshotter::startAutoTrigger()
{
  if (!auto_trigger_enabled_) {
    RCLCPP_INFO(get_logger(), "Automatic snapshot capture is disabled.");
    return;
  }

  auto_trigger_ready_after_s_ = steadySeconds() + auto_trigger_startup_grace_s_;

  for (const auto & rule : auto_trigger_rules_) {
    rule->subscription = subscribeAutoTriggerEndpoint(rule, rule->kind, rule->topic, false);
    if (!rule->scope_topic.empty()) {
      rule->scope_subscription =
        subscribeAutoTriggerEndpoint(rule, rule->scope_kind, rule->scope_topic, true);
    }
    RCLCPP_INFO(
      get_logger(),
      "Automatic snapshot rule '%s' watching %s (hold %.1fs, %d occurrence(s)%s)",
      rule->name.c_str(), rule->topic.c_str(), rule->hold_s, rule->min_occurrences,
      rule->scope_topic.empty() ? "" : (", counted per " + rule->scope_topic).c_str());
  }

  auto_trigger_timer_ = create_wall_timer(
    std::chrono::duration(200ms),
    std::bind(&Snapshotter::evaluateAutoTriggers, this));
  auto_trigger_worker_ = std::thread(&Snapshotter::runAutoTriggerWorker, this);

  RCLCPP_INFO(
    get_logger(),
    "Automatic snapshot capture armed in %.0fs: %zu rule(s), cooldown %.0fs, output %s",
    auto_trigger_startup_grace_s_, auto_trigger_rules_.size(), auto_trigger_cooldown_s_,
    auto_trigger_directory_.c_str());
}

void Snapshotter::stopAutoTrigger()
{
  {
    std::lock_guard<std::mutex> lock(auto_trigger_lock_);
    auto_trigger_shutdown_ = true;
  }
  auto_trigger_cv_.notify_all();
  if (auto_trigger_worker_.joinable()) {
    auto_trigger_worker_.join();
  }
  auto_trigger_timer_.reset();
  for (const auto & rule : auto_trigger_rules_) {
    rule->subscription.reset();
    rule->scope_subscription.reset();
  }
}

rclcpp::SubscriptionBase::SharedPtr Snapshotter::subscribeAutoTriggerEndpoint(
  const std::shared_ptr<AutoTriggerRule> & rule, const Kind_t kind, const std::string & topic,
  const bool scope)
{
  // Volatile on purpose. These status publishers are transient_local, so a
  // latched fault from the previous run would otherwise spend the buffer on
  // stale evidence the moment this node starts.
  const auto qos = rclcpp::QoS{10}.reliable().durability_volatile();

  switch (kind) {
    case Kind_t::kModuleState:
      return create_subscription<avg_msgs::msg::ModuleState>(
        topic, qos,
        [this, rule, scope](avg_msgs::msg::ModuleState::ConstSharedPtr msg) {
          if (scope) {
            noteAutoTriggerScope(rule, msg->operating_state);
          } else {
            onAutoTriggerModuleState(rule, *msg);
          }
        });
    case Kind_t::kSystemStatus:
      return create_subscription<avg_msgs::msg::SystemStatus>(
        topic, qos,
        [this, rule, scope](avg_msgs::msg::SystemStatus::ConstSharedPtr msg) {
          if (scope) {
            std::string scope_state;
            for (const auto & module : msg->modules) {
              if (rule->scope_reset_states.count(module.operating_state) > 0) {
                scope_state = module.operating_state;
                break;
              }
            }
            noteAutoTriggerScope(rule, scope_state);
          } else {
            onAutoTriggerSystemStatus(rule, *msg);
          }
        });
    case Kind_t::kServiceState:
    default:
      return create_subscription<avg_msgs::msg::AvgServiceState>(
        topic, qos,
        [this, rule, scope](avg_msgs::msg::AvgServiceState::ConstSharedPtr msg) {
          if (scope) {
            noteAutoTriggerScope(rule, msg->state_name);
          } else {
            onAutoTriggerServiceState(rule, *msg);
          }
        });
  }
}

void Snapshotter::onAutoTriggerServiceState(
  const std::shared_ptr<AutoTriggerRule> & rule, const avg_msgs::msg::AvgServiceState & msg)
{
  const bool matched = rule->operating_states.count(msg.state_name) > 0;
  noteAutoTriggerMatch(rule, matched, matched ? ("service=" + msg.state_name) : std::string{});
}

void Snapshotter::noteAutoTriggerScope(
  const std::shared_ptr<AutoTriggerRule> & rule, const std::string & scope_state)
{
  int cleared = 0;
  bool did_reset = false;
  {
    std::lock_guard<std::mutex> lock(auto_trigger_lock_);
    // Trigger on a change of scope state, not on being inside one. A topic
    // republishing the same state must not keep clearing a count that is
    // still accumulating inside that episode, while two different reset
    // states in a row must still open two episodes.
    const bool unchanged = rule->scope_state_valid && rule->last_scope_state == scope_state;
    rule->last_scope_state = scope_state;
    rule->scope_state_valid = true;
    rule->scope_active = rule->scope_active_states.empty() ||
      rule->scope_active_states.count(scope_state) > 0;
    if (unchanged || rule->scope_reset_states.count(scope_state) == 0) {
      return;
    }
    did_reset = true;
    cleared = rule->occurrences;
    rule->occurrences = 0;
    rule->fire_pending = false;
    rule->pending_detail.clear();
  }
  if (did_reset) {
    RCLCPP_INFO(
      get_logger(), "Automatic snapshot rule '%s' count reset at %s=%s (was %d).",
      rule->name.c_str(), rule->scope_topic.c_str(), scope_state.c_str(), cleared);
  }
}

void Snapshotter::onAutoTriggerModuleState(
  const std::shared_ptr<AutoTriggerRule> & rule, const avg_msgs::msg::ModuleState & msg)
{
  const bool state_match = !rule->operating_states.empty() &&
    rule->operating_states.count(msg.operating_state) > 0;
  const bool level_match = rule->min_level >= 0 &&
    static_cast<int>(msg.level) >= rule->min_level;

  std::string detail;
  if (state_match || level_match) {
    detail = "module=" + (msg.module_name.empty() ? rule->topic : msg.module_name) +
      " state=" + msg.operating_state +
      " level=" + std::to_string(static_cast<int>(msg.level));
  }
  noteAutoTriggerMatch(rule, state_match || level_match, detail);
}

void Snapshotter::onAutoTriggerSystemStatus(
  const std::shared_ptr<AutoTriggerRule> & rule, const avg_msgs::msg::SystemStatus & msg)
{
  // Cap the recorded detail: it becomes a log line, and an aggregate can carry
  // dozens of modules during a cold start.
  constexpr std::size_t kMaxReportedModules = 5;

  std::string detail;
  std::size_t offenders = 0;
  for (const auto & module : msg.modules) {
    if (!rule->module_names.empty() && rule->module_names.count(module.module_name) == 0) {
      continue;
    }
    const bool level_match = rule->min_level >= 0 &&
      static_cast<int>(module.level) >= rule->min_level;
    const bool state_match = !rule->operating_states.empty() &&
      rule->operating_states.count(module.operating_state) > 0;
    if (!level_match && !state_match) {
      continue;
    }
    if (offenders < kMaxReportedModules) {
      detail += (detail.empty() ? "modules=" : ",") + module.module_name + ":" +
        std::to_string(static_cast<int>(module.level));
    }
    ++offenders;
  }
  if (offenders > kMaxReportedModules) {
    detail += ",+" + std::to_string(offenders - kMaxReportedModules);
  }

  const bool aggregate_match = rule->on_system_not_ok && !msg.system_ok;
  if (aggregate_match) {
    detail = detail.empty() ? "system_ok=false" : ("system_ok=false " + detail);
  }
  noteAutoTriggerMatch(rule, aggregate_match || offenders > 0, detail);
}

void Snapshotter::noteAutoTriggerMatch(
  const std::shared_ptr<AutoTriggerRule> & rule, const bool matched, const std::string & detail)
{
  const double now_s = steadySeconds();

  bool armed_now = false;
  {
    std::lock_guard<std::mutex> lock(auto_trigger_lock_);
    if (matched) {
      if (!rule->matching) {
        rule->matching = true;
        rule->matching_since_s = now_s;
        // Count the rising edge here rather than on the evaluation tick. A
        // margin contact that the crab recovery clears in well under the tick
        // period is still a contact, and it must not be missed.
        //
        // Only while armed: contacts seen before the topic ever reported
        // healthy belong to a cold start, and letting them accumulate would
        // make the first contact after arming look like the Nth.
        // Only while armed and in scope. A contact inside a campsite or the
        // charger bay is expected - those areas sit outside the road lanelets
        // by design and the maneuver controllers are allowed to cross that
        // boundary - so only the road legs may contribute to the count.
        if (rule->armed && rule->scope_active) {
          ++rule->occurrences;
          if (rule->hold_s <= 0.0 && rule->occurrences >= rule->min_occurrences) {
            rule->fire_pending = true;
            rule->pending_detail =
              detail + " occurrences=" + std::to_string(rule->occurrences) + "/" +
              std::to_string(rule->min_occurrences);
          }
        }
      }
      rule->detail = detail;
      return;
    }

    if (rule->matching || !rule->clear_since_valid) {
      rule->clear_since_s = now_s;
      rule->clear_since_valid = true;
    }
    rule->matching = false;
    rule->detail.clear();
    // Arm on a healthy report that has held long enough. This is both the
    // startup guard and the re-arm edge: one continuous fault must not spend
    // one buffer per cooldown window for as long as it persists.
    if (!rule->armed && now_s - rule->clear_since_s >= rule->require_healthy_s) {
      rule->armed = true;
      armed_now = true;
    }
  }

  if (armed_now) {
    RCLCPP_INFO(
      get_logger(), "Automatic snapshot rule '%s' armed: %s reported healthy.",
      rule->name.c_str(), rule->topic.c_str());
  }
}

void Snapshotter::evaluateAutoTriggers()
{
  const double now_s = steadySeconds();

  AutoTriggerCapture queued;
  bool fired = false;
  std::vector<std::string> dropped;
  {
    std::lock_guard<std::mutex> lock(auto_trigger_lock_);
    if (now_s < auto_trigger_ready_after_s_) {
      return;
    }

    if (auto_trigger_fired_ && now_s - auto_trigger_last_fire_s_ < auto_trigger_cooldown_s_) {
      // A fire latched inside the cooldown describes a buffer that the
      // previous capture already cleared. Drop it here rather than letting it
      // write stale evidence once the window expires.
      for (const auto & rule : auto_trigger_rules_) {
        if (rule->fire_pending) {
          rule->fire_pending = false;
          rule->pending_detail.clear();
          dropped.push_back(rule->name);
        }
      }
    } else {
      for (const auto & rule : auto_trigger_rules_) {
        // A rule with hold_s promotes a sustained condition here; a counting
        // rule has already latched fire_pending on its Nth rising edge.
        if (!rule->fire_pending && rule->armed && rule->hold_s > 0.0 && rule->matching &&
          now_s - rule->matching_since_s >= rule->hold_s &&
          rule->occurrences >= rule->min_occurrences)
        {
          rule->fire_pending = true;
          rule->pending_detail = rule->detail;
        }
        if (!rule->fire_pending || !rule->armed) {
          continue;
        }
        rule->fire_pending = false;
        rule->armed = false;
        // Start the next episode from zero: this capture covers the
        // occurrences that led to it.
        rule->occurrences = 0;
        // The cooldown starts at the decision rather than at completion: the
        // buffer is spent either way, and a write may take minutes.
        auto_trigger_fired_ = true;
        auto_trigger_last_fire_s_ = now_s;
        queued = AutoTriggerCapture{rule->name, rule->pending_detail};
        rule->pending_detail.clear();
        auto_trigger_pending_.push_back(queued);
        fired = true;
        // One capture per cooldown window: a fault cascade trips several
        // rules at once and they would all describe the same buffer.
        break;
      }
    }
  }

  for (const auto & name : dropped) {
    RCLCPP_WARN(
      get_logger(),
      "Automatic snapshot rule '%s' tripped inside the cooldown window; "
      "the previous capture already cleared that buffer.",
      name.c_str());
  }
  if (!fired) {
    return;
  }
  // Logged before the worker pauses recording so /rosout carries the reason
  // inside the bag that is about to be written.
  RCLCPP_WARN(
    get_logger(), "Automatic snapshot rule '%s' fired: %s",
    queued.rule_name.c_str(), queued.detail.c_str());
  auto_trigger_cv_.notify_one();
}

void Snapshotter::runAutoTriggerWorker()
{
  while (true) {
    AutoTriggerCapture capture;
    {
      std::unique_lock<std::mutex> lock(auto_trigger_lock_);
      auto_trigger_cv_.wait(
        lock, [this] {
          return auto_trigger_shutdown_ || !auto_trigger_pending_.empty();
        });
      if (auto_trigger_shutdown_) {
        return;
      }
      capture = auto_trigger_pending_.front();
      auto_trigger_pending_.pop_front();
    }
    captureAutoTrigger(capture);
  }
}

void Snapshotter::autoTriggerStorageBudget(
  const std::string & probe_path, uint64_t & reserve_bytes, uint64_t & max_bytes) const
{
  reserve_bytes = 0;
  max_bytes = 0;

  std::error_code error;
  auto probe = std::filesystem::path(probe_path);
  while (!probe.empty() && !std::filesystem::exists(probe, error) &&
    probe.has_parent_path() && probe.parent_path() != probe)
  {
    probe = probe.parent_path();
  }

  const auto space = std::filesystem::space(probe, error);
  if (error) {
    // Leave both budgets at zero. writeSnapshot still refuses to start when
    // its own reserve check fails, so this only loses the size fitting.
    return;
  }

  reserve_bytes = std::max<uint64_t>(
    auto_trigger_minimum_free_mb_ * 1000000ULL,
    static_cast<uint64_t>(
      static_cast<double>(space.capacity) * auto_trigger_minimum_free_ratio_));
  const uint64_t writable_bytes =
    space.available > reserve_bytes ? space.available - reserve_bytes : 0ULL;
  max_bytes = static_cast<uint64_t>(
    static_cast<double>(writable_bytes) / auto_trigger_size_safety_factor_);
}

void Snapshotter::captureAutoTrigger(const AutoTriggerCapture & capture)
{
  std::error_code error;
  std::filesystem::create_directories(auto_trigger_directory_, error);
  if (error) {
    RCLCPP_ERROR(
      get_logger(), "Automatic snapshot '%s' aborted: cannot create %s (%s)",
      capture.rule_name.c_str(), auto_trigger_directory_.c_str(), error.message().c_str());
    return;
  }

  uint64_t reserve_bytes = 0;
  uint64_t max_bytes = 0;
  autoTriggerStorageBudget(auto_trigger_directory_, reserve_bytes, max_bytes);
  if (reserve_bytes > 0 && max_bytes == 0) {
    // A zero budget would read as "no limit" in estimateBuffers, so refuse
    // here rather than letting the request bypass the size fitting.
    RCLCPP_ERROR(
      get_logger(),
      "Automatic snapshot '%s' aborted: %s is below its %lu MB storage reserve.",
      capture.rule_name.c_str(), auto_trigger_directory_.c_str(),
      static_cast<unsigned long>(auto_trigger_minimum_free_mb_));  // NOLINT(runtime/int)
    return;
  }

  auto request = std::make_shared<TriggerSnapshot::Request>();
  auto response = std::make_shared<TriggerSnapshot::Response>();

  // postfixFilename appends the local datetime and the .bag extension to a
  // name that does not already end in .bag, hence the trailing separator.
  request->filename =
    (std::filesystem::path(auto_trigger_directory_) /
    (auto_trigger_prefix_ + "_" + capture.rule_name + "_")).string();
  request->max_bytes = max_bytes;
  request->minimum_free_bytes = reserve_bytes;
  if (auto_trigger_lookback_s_ > 0.0) {
    request->start_time = static_cast<builtin_interfaces::msg::Time>(
      now() - rclcpp::Duration::from_seconds(auto_trigger_lookback_s_));
  }

  writeSnapshot(request, response, "auto:" + capture.rule_name);

  if (response->success) {
    RCLCPP_WARN(
      get_logger(), "Automatic snapshot '%s' written: %s",
      capture.rule_name.c_str(), response->message.c_str());
  } else {
    RCLCPP_ERROR(
      get_logger(), "Automatic snapshot '%s' failed: %s",
      capture.rule_name.c_str(), response->message.c_str());
  }
}

void Snapshotter::parseOffloadParams()
{
  offload_enabled_ = declare_parameter<bool>("offload.enabled", false);
  offload_host_ = declare_parameter<std::string>("offload.host", "");
  offload_user_ = declare_parameter<std::string>("offload.user", "");
  offload_port_ = static_cast<int>(declare_parameter<int64_t>("offload.port", 22));
  offload_remote_directory_ =
    declare_parameter<std::string>("offload.remote_directory", "");
  offload_identity_file_ = declare_parameter<std::string>("offload.identity_file", "");
  offload_remove_local_ = declare_parameter<bool>("offload.remove_local_after_transfer", true);
  offload_connect_timeout_s_ =
    static_cast<int>(declare_parameter<int64_t>("offload.connect_timeout_s", 10));
  offload_transfer_timeout_s_ =
    static_cast<int>(declare_parameter<int64_t>("offload.transfer_timeout_s", 1800));
  offload_retries_ = static_cast<int>(declare_parameter<int64_t>("offload.retries", 2));
  offload_retry_delay_s_ =
    static_cast<int>(declare_parameter<int64_t>("offload.retry_delay_s", 30));

  if (!offload_enabled_) {
    return;
  }
  if (offload_host_.empty()) {
    throw std::invalid_argument("offload.enabled requires offload.host");
  }
  if (offload_remote_directory_.empty() || offload_remote_directory_.front() != '/') {
    throw std::invalid_argument(
            "offload.enabled requires an absolute offload.remote_directory");
  }
  if (offload_port_ < 1 || offload_port_ > 65535) {
    throw std::invalid_argument("offload.port must be in [1, 65535]");
  }
  if (offload_connect_timeout_s_ < 1 || offload_transfer_timeout_s_ < 1) {
    throw std::invalid_argument("offload timeouts must be at least one second");
  }
  if (offload_retries_ < 0 || offload_retry_delay_s_ < 0) {
    throw std::invalid_argument("offload.retries and retry_delay_s must not be negative");
  }
}

void Snapshotter::startOffload()
{
  if (!offload_enabled_) {
    RCLCPP_INFO(get_logger(), "Snapshot offload is disabled; bags stay on this machine.");
    return;
  }
  offload_worker_ = std::thread(&Snapshotter::runOffloadWorker, this);
  RCLCPP_INFO(
    get_logger(), "Snapshot offload enabled: %s%s:%d%s (%s local copy after transfer)",
    offload_user_.empty() ? "" : (offload_user_ + "@").c_str(), offload_host_.c_str(),
    offload_port_, offload_remote_directory_.c_str(),
    offload_remove_local_ ? "removing" : "keeping");
}

void Snapshotter::stopOffload()
{
  {
    std::lock_guard<std::mutex> lock(offload_lock_);
    offload_shutdown_ = true;
  }
  offload_cv_.notify_all();
  if (offload_worker_.joinable()) {
    offload_worker_.join();
  }
}

void Snapshotter::enqueueOffload(const std::string & local_path, const std::string & origin)
{
  if (!offload_enabled_ || local_path.empty()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(offload_lock_);
    if (offload_shutdown_) {
      return;
    }
    offload_pending_.push_back(OffloadRequest{local_path, origin});
  }
  offload_cv_.notify_one();
}

std::vector<std::string> Snapshotter::sshOptionArgs() const
{
  // BatchMode refuses every prompt. An unattended robot must fail loudly and
  // keep the local bag rather than block forever on a password or a host-key
  // question, so the key has to be installed ahead of time.
  std::vector<std::string> args{
    "-p", std::to_string(offload_port_),
    "-o", "BatchMode=yes",
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", "ConnectTimeout=" + std::to_string(offload_connect_timeout_s_)};
  if (!offload_identity_file_.empty()) {
    args.push_back("-i");
    args.push_back(offload_identity_file_);
  }
  return args;
}

int Snapshotter::runProcess(
  const std::vector<std::string> & argv, const int timeout_s, std::string & error)
{
  if (argv.empty()) {
    error = "empty command";
    return -1;
  }

  std::vector<char *> c_argv;
  c_argv.reserve(argv.size() + 1U);
  for (const auto & argument : argv) {
    c_argv.push_back(const_cast<char *>(argument.c_str()));
  }
  c_argv.push_back(nullptr);

  const pid_t pid = fork();
  if (pid < 0) {
    error = "fork failed";
    return -1;
  }
  if (pid == 0) {
    // Child. execvp takes the argument vector directly, so no shell parses
    // these strings and a bag path can never become part of a command.
    execvp(c_argv[0], c_argv.data());
    _exit(127);
  }

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(std::max(1, timeout_s));
  while (true) {
    int status = 0;
    const pid_t finished = waitpid(pid, &status, WNOHANG);
    if (finished == pid) {
      if (WIFEXITED(status)) {
        return WEXITSTATUS(status);
      }
      error = "terminated by signal";
      return -1;
    }
    if (finished < 0) {
      error = "waitpid failed";
      return -1;
    }

    bool aborting = false;
    {
      std::lock_guard<std::mutex> lock(offload_lock_);
      aborting = offload_shutdown_;
    }
    const bool timed_out = std::chrono::steady_clock::now() > deadline;
    if (aborting || timed_out) {
      // Never leave a stalled transfer holding up shutdown.
      kill(pid, SIGKILL);
      waitpid(pid, nullptr, 0);
      error = aborting ? "aborted at shutdown" : "timed out";
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

bool Snapshotter::offloadOnce(const OffloadRequest & request, std::string & error)
{
  const std::string destination =
    offload_user_.empty() ? offload_host_ : (offload_user_ + "@" + offload_host_);
  const auto options = sshOptionArgs();

  // Create the target directory first: a missing one is the most common
  // failure and rsync's error for it is far less obvious.
  std::vector<std::string> mkdir_argv{"ssh"};
  mkdir_argv.insert(mkdir_argv.end(), options.begin(), options.end());
  mkdir_argv.push_back(destination);
  mkdir_argv.push_back("mkdir");
  mkdir_argv.push_back("-p");
  mkdir_argv.push_back(offload_remote_directory_);

  int status = runProcess(mkdir_argv, offload_connect_timeout_s_ + 10, error);
  if (status != 0) {
    if (error.empty()) {
      error = "remote mkdir exited with status " + std::to_string(status);
    }
    return false;
  }

  // rsync splits -e itself, which is why the transport is one string. It
  // carries no bag path, so nothing user-supplied is ever re-parsed.
  std::string ssh_transport = "ssh";
  for (const auto & option : options) {
    ssh_transport += " " + option;
  }

  // A rosbag2 bag is a directory of metadata plus database files. The source
  // has no trailing separator on purpose: that copies the bag directory into
  // the remote folder rather than spilling its contents there.
  std::vector<std::string> rsync_argv{
    "rsync", "-a", "--partial", "-e", ssh_transport, request.local_path,
    destination + ":" + offload_remote_directory_ + "/"};

  status = runProcess(rsync_argv, offload_transfer_timeout_s_, error);
  if (status != 0) {
    if (error.empty()) {
      error = "rsync exited with status " + std::to_string(status);
    }
    return false;
  }
  return true;
}

void Snapshotter::runOffloadWorker()
{
  while (true) {
    OffloadRequest request;
    {
      std::unique_lock<std::mutex> lock(offload_lock_);
      offload_cv_.wait(
        lock, [this] {
          return offload_shutdown_ || !offload_pending_.empty();
        });
      if (offload_shutdown_) {
        return;
      }
      request = offload_pending_.front();
      offload_pending_.pop_front();
    }

    std::error_code file_error;
    if (!std::filesystem::exists(request.local_path, file_error)) {
      RCLCPP_WARN(
        get_logger(), "Snapshot offload skipped: %s no longer exists.",
        request.local_path.c_str());
      continue;
    }

    bool transferred = false;
    std::string error;
    for (int attempt = 0; attempt <= offload_retries_; ++attempt) {
      error.clear();
      if (offloadOnce(request, error)) {
        transferred = true;
        break;
      }
      RCLCPP_WARN(
        get_logger(), "Snapshot offload attempt %d/%d for %s failed: %s",
        attempt + 1, offload_retries_ + 1, request.local_path.c_str(), error.c_str());

      if (attempt == offload_retries_) {
        break;
      }
      // Wait between attempts, but wake immediately on shutdown.
      std::unique_lock<std::mutex> lock(offload_lock_);
      offload_cv_.wait_for(
        lock, std::chrono::seconds(std::max(0, offload_retry_delay_s_)),
        [this] {return offload_shutdown_;});
      if (offload_shutdown_) {
        break;
      }
    }

    if (!transferred) {
      // Keep the local bag. A failed transfer must never be able to destroy
      // the only copy of the evidence.
      RCLCPP_ERROR(
        get_logger(),
        "Snapshot offload gave up on %s (%s); the bag stays on this machine.",
        request.local_path.c_str(), request.origin.c_str());
      continue;
    }

    if (!offload_remove_local_) {
      RCLCPP_INFO(
        get_logger(), "Snapshot %s (%s) copied to %s:%s",
        request.local_path.c_str(), request.origin.c_str(), offload_host_.c_str(),
        offload_remote_directory_.c_str());
      continue;
    }

    std::filesystem::remove_all(request.local_path, file_error);
    if (file_error) {
      RCLCPP_WARN(
        get_logger(),
        "Snapshot %s reached %s:%s but the local copy could not be removed: %s",
        request.local_path.c_str(), offload_host_.c_str(),
        offload_remote_directory_.c_str(), file_error.message().c_str());
      continue;
    }
    RCLCPP_INFO(
      get_logger(), "Snapshot %s (%s) moved to %s:%s",
      request.local_path.c_str(), request.origin.c_str(), offload_host_.c_str(),
      offload_remote_directory_.c_str());
  }
}

SnapshotterClient::SnapshotterClient(const rclcpp::NodeOptions & options)
: rclcpp::Node("snapshotter_client", options)
{
  std::string action_str{};

  SnapshotterClientOptions opts{};

  try {
    action_str = declare_parameter<std::string>("action_type");
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "action_type parameter is missing or of incorrect type.");
    throw ex;
  }

  if (action_str == "trigger_write") {
    opts.action_ = SnapshotterClientOptions::TRIGGER_WRITE;
  } else if (action_str == "resume") {
    opts.action_ = SnapshotterClientOptions::RESUME;
  } else if (action_str == "pause") {
    opts.action_ = SnapshotterClientOptions::PAUSE;
  } else {
    RCLCPP_ERROR(get_logger(), "action_type must be one of: trigger_write, resume, or pause");
    throw std::invalid_argument{"Invalid value for action_type parameter."};
  }

  std::vector<std::string> topic_names{};

  try {
    topic_names = declare_parameter<std::vector<std::string>>("topics");
  } catch (const rclcpp::ParameterTypeException & ex) {
    if (std::string{ex.what()}.find("not set") == std::string::npos) {
      RCLCPP_ERROR(get_logger(), "topics must be an array of strings.");
      throw ex;
    }
  }

  if (topic_names.size() > 0) {
    for (const auto & topic : topic_names) {
      std::string prefix = "topic_details." + topic;
      std::string topic_type{};

      try {
        topic_type = declare_parameter<std::string>(prefix + ".type");
      } catch (const rclcpp::ParameterTypeException & ex) {
        if (std::string{ex.what()}.find("not set") == std::string::npos) {
          RCLCPP_ERROR(get_logger(), "Topic type must be a string.");
        } else {
          RCLCPP_ERROR(get_logger(), "Topic %s is missing a type.", topic.c_str());
        }

        throw ex;
      }

      TopicDetails details{};
      details.name = topic;
      details.type = topic_type;
      opts.topics_.push_back(details);
    }
  }

  try {
    opts.filename_ = declare_parameter<std::string>("filename");
  } catch (const rclcpp::ParameterTypeException & ex) {
    if (opts.action_ == SnapshotterClientOptions::TRIGGER_WRITE &&
      std::string{ex.what()}.find("not set") == std::string::npos)
    {
      RCLCPP_ERROR(get_logger(), "filename must be a string.");
      throw ex;
    }
  }

  try {
    opts.prefix_ = declare_parameter<std::string>("prefix");
  } catch (const rclcpp::ParameterTypeException & ex) {
    if (opts.action_ == SnapshotterClientOptions::TRIGGER_WRITE &&
      std::string{ex.what()}.find("not set") == std::string::npos)
    {
      RCLCPP_ERROR(get_logger(), "prefix must be a string.");
      throw ex;
    }
  }

  if (opts.action_ == SnapshotterClientOptions::TRIGGER_WRITE && opts.topics_.size() == 0) {
    RCLCPP_INFO(get_logger(), "No topics provided - logging all topics.");
    RCLCPP_WARN(get_logger(), "Logging all topics is very memory-intensive.");
  }

  setSnapshotterClientOptions(opts);
}

void SnapshotterClient::setSnapshotterClientOptions(const SnapshotterClientOptions & opts)
{
  if (opts.action_ == SnapshotterClientOptions::TRIGGER_WRITE) {
    auto client = create_client<TriggerSnapshot>("trigger_snapshot");
    if (!client->service_is_ready()) {
      throw std::runtime_error{
              "Service trigger_snapshot is not ready. "
              "Is snapshot running in this namespace?"
      };
    }

    auto req = std::make_shared<TriggerSnapshot::Request>();

    for (const auto & topic : opts.topics_) {
      req->topics.push_back(topic.asMessage());
    }

    // Prefix mode
    if (opts.filename_.empty()) {
      req->filename = opts.prefix_;
      size_t ind = req->filename.rfind(".bag");
      if (ind != string::npos && ind == req->filename.size() - 4) {
        req->filename.erase(ind);
      }
    } else {
      req->filename = opts.filename_;
      size_t ind = req->filename.rfind(".bag");
      if (ind == string::npos || ind != req->filename.size() - 4) {
        req->filename += ".bag";
      }
    }

    // Resolve filename relative to clients working directory to avoid confusion
    // Special case of no specified file, ensure still in working directory of client
    if (req->filename.empty()) {
      req->filename = "./";
    }
    std::filesystem::path p(std::filesystem::absolute(req->filename));
    req->filename = p.string();

    auto result_future = client->async_send_request(req);
    auto future_result =
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);

    if (future_result == rclcpp::FutureReturnCode::SUCCESS) {
      auto result = result_future.get();
      RCLCPP_INFO(
        get_logger(),
        "Service returned: [%s] %s",
        (result->success ? "SUCCESS" : "FAILURE"),
        result->message.c_str()
      );
    } else {
      RCLCPP_ERROR(get_logger(), "Calling the service failed.");
    }

    return;
  } else if (  // NOLINT
    opts.action_ == SnapshotterClientOptions::PAUSE ||
    opts.action_ == SnapshotterClientOptions::RESUME)
  {
    auto client = create_client<SetBool>("enable_snapshot");
    if (!client->service_is_ready()) {
      throw std::runtime_error{
              "Service enable_snapshot does not exist. "
              "Is snapshot running in this namespace?"
      };
    }

    auto req = std::make_shared<SetBool::Request>();
    req->data = (opts.action_ == SnapshotterClientOptions::RESUME);

    auto result_future = client->async_send_request(req);
    auto future_result =
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);

    if (future_result == rclcpp::FutureReturnCode::SUCCESS) {
      auto result = result_future.get();
      RCLCPP_INFO(
        get_logger(),
        "Service returned: [%s] %s",
        (result->success ? "SUCCESS" : "FAILURE"),
        result->message.c_str()
      );
    } else {
      RCLCPP_ERROR(get_logger(), "Calling the service failed.");
    }

    return;
  } else {
    throw std::runtime_error{"Invalid options received."};
  }
}

}  // namespace camrod_snapshot

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(camrod_snapshot::Snapshotter)
RCLCPP_COMPONENTS_REGISTER_NODE(camrod_snapshot::SnapshotterClient)
