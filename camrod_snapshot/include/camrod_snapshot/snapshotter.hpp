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

#ifndef CAMROD_SNAPSHOT__SNAPSHOTTER_HPP_
#define CAMROD_SNAPSHOT__SNAPSHOTTER_HPP_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <shared_mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <avg_msgs/msg/avg_service_state.hpp>
#include <avg_msgs/msg/module_state.hpp>
#include <avg_msgs/msg/system_status.hpp>
#include <avg_msgs/msg/topic_details.hpp>
#include <avg_msgs/srv/configure_snapshot_topics.hpp>
#include <avg_msgs/srv/estimate_snapshot.hpp>
#include <avg_msgs/srv/trigger_snapshot.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace camrod_snapshot
{
using namespace std::chrono_literals;  // NOLINT
using DetailsMsg = avg_msgs::msg::TopicDetails;

struct TopicDetails
{
  std::string name;
  std::string type;

  TopicDetails() {}

  TopicDetails(std::string name, std::string type)
  : name(name), type(type) {}

  bool operator==(const TopicDetails & t) const
  {
    return name == t.name && type == t.type;
  }

  bool operator<(const TopicDetails & t) const
  {
    return t.name < name || (t.name == name && t.type < type);
  }

  bool operator>(const TopicDetails & t) const
  {
    return t.name > name || (t.name == name && t.type > type);
  }

  DetailsMsg asMessage() const
  {
    DetailsMsg msg{};
    msg.name = name;
    msg.type = type;
    return msg;
  }
};

class Snapshotter;

/* Configuration for a single topic in the Snapshotter node. Holds
 * the buffer limits for a topic by duration (time difference between newest and oldest message)
 * and memory usage, in bytes.
 */
struct SnapshotterTopicOptions
{
  // When the value of duration_limit_, do not truncate the buffer
  // no matter how large the duration is
  static const rclcpp::Duration NO_DURATION_LIMIT;
  // When the value of memory_limit_, do not trunctate the buffer
  // no matter how much memory it consumes (DANGROUS)
  static const int32_t NO_MEMORY_LIMIT;
  // When the value of duration_limit_, inherit the limit from
  // the node's configured default
  static const rclcpp::Duration INHERIT_DURATION_LIMIT;
  // When the value of memory_limit_, inherit the limit from
  // the node's configured default
  static const int32_t INHERIT_MEMORY_LIMIT;
  // When the value of min_interval_, store every received message (no throttling)
  static const rclcpp::Duration NO_MIN_INTERVAL;
  // When the value of min_interval_, inherit the interval from
  // the node's configured default
  static const rclcpp::Duration INHERIT_MIN_INTERVAL;

  // Maximum difference in time from newest and oldest message in
  // buffer before older messages are removed
  rclcpp::Duration duration_limit_;
  // Maximum memory usage of the buffer before older messages are removed
  int32_t memory_limit_;
  // Minimum time between two stored messages; messages arriving sooner
  // than this after the last stored one are dropped (throttling)
  rclcpp::Duration min_interval_;

  SnapshotterTopicOptions(
    rclcpp::Duration duration_limit = INHERIT_DURATION_LIMIT,
    int32_t memory_limit = INHERIT_MEMORY_LIMIT,
    rclcpp::Duration min_interval = INHERIT_MIN_INTERVAL);
};

/* Configuration for the Snapshotter node. Contains default limits for memory and duration
 * and a map of topics to their limits which may override the defaults.
 */
struct SnapshotterOptions
{
  // Duration limit to use for a topic's buffer if one is not specified
  rclcpp::Duration default_duration_limit_;
  // Memory limit to use for a topic's buffer if one is not specified
  int32_t default_memory_limit_;
  // Minimum interval between stored messages to use for a topic if one is not specified
  rclcpp::Duration default_min_interval_;
  // Flag if all topics should be recorded
  bool all_topics_;

  typedef std::map<TopicDetails, SnapshotterTopicOptions> topics_t;
  // Provides list of topics to snapshot and their limit configurations
  topics_t topics_;

  SnapshotterOptions(
    rclcpp::Duration default_duration_limit = rclcpp::Duration(30s),
    int32_t default_memory_limit = -1,
    rclcpp::Duration default_min_interval = rclcpp::Duration(0s));

  // Add a new topic to the configuration, returns false if the topic was already present
  bool addTopic(
    const TopicDetails & topic_details,
    rclcpp::Duration duration_limit = SnapshotterTopicOptions::INHERIT_DURATION_LIMIT,
    int32_t memory_limit = SnapshotterTopicOptions::INHERIT_MEMORY_LIMIT,
    rclcpp::Duration min_interval = SnapshotterTopicOptions::INHERIT_MIN_INTERVAL);
};

/* Stores a buffered message of an ambiguous type and it's associated metadata (time of arrival),
 * for later writing to disk
 */
struct SnapshotMessage
{
  SnapshotMessage(
    std::shared_ptr<const rclcpp::SerializedMessage> _msg,
    rclcpp::Time _time);
  std::shared_ptr<const rclcpp::SerializedMessage> msg;
  // ROS time when messaged arrived (does not use header stamp)
  rclcpp::Time time;
};

/* Stores a queue of buffered messages for a single topic ensuring
 * that the duration and memory limits are respected by truncating
 * as needed on push() operations.
 */
class MessageQueue
{
  friend Snapshotter;

private:
  // Logger for outputting ROS logging messages
  rclcpp::Logger logger_;
  // Locks access to size_ and queue_
  std::mutex lock;
  // Stores limits on buffer size and duration
  SnapshotterTopicOptions options_;
  // Current total size of the queue, in bytes
  int64_t size_;
  typedef std::deque<SnapshotMessage> queue_t;
  queue_t queue_;
  // Subscriber to the callback which uses this queue
  std::shared_ptr<rclcpp::GenericSubscription> sub_;

public:
  explicit MessageQueue(const SnapshotterTopicOptions & options, const rclcpp::Logger & logger);
  // Add a new message to the internal queue if possible, truncating the front
  // of the queue as needed to enforce limits
  void push(const SnapshotMessage & msg);
  // Removes the message at the front of the queue (oldest) and returns it
  SnapshotMessage pop();
  // Returns the time difference between back and front of queue, or 0 if size <= 1
  rclcpp::Duration duration() const;
  // Clear internal buffer
  void clear();
  // Store the subscriber for this topic's queue internaly so it is not deleted
  void setSubscriber(std::shared_ptr<rclcpp::GenericSubscription> sub);
  typedef std::pair<queue_t::const_iterator, queue_t::const_iterator> range_t;
  // Get a begin and end iterator into the buffer respecting the start and
  // end timestamp constraints
  range_t rangeFromTimes(const rclcpp::Time & start, const rclcpp::Time & end);

  // Return the total message size including the meta-information
  int64_t getMessageSize(SnapshotMessage const & msg) const;

private:
  // Internal push whitch does not obtain lock
  void _push(SnapshotMessage const & msg);
  // Internal pop which does not obtain lock
  SnapshotMessage _pop();
  // Internal clear which does not obtain lock
  void _clear();
  // Truncate front of queue as needed to fit a new message of specified size and time.
  // Returns False if this is impossible.
  bool preparePush(int32_t size, rclcpp::Time const & time);
};

// Snapshotter node. Maintains a circular buffer of the most recent messages
// from configured topics while enforcing limits on memory and duration.
// The node can be triggered to write some or all of these buffers to a bag
// file via a service call. Useful in live testing scenerios where interesting
// data may be produced before a user has the oppurtunity to "rosbag record" the data.
class Snapshotter : public rclcpp::Node
{
public:
  explicit Snapshotter(const rclcpp::NodeOptions & options);
  ~Snapshotter();

private:
  // Subscribe queue size for each topic
  static const int QUEUE_SIZE;
  SnapshotterOptions options_;
  // Maximum duration of each database file inside one snapshot bag, in seconds.
  uint64_t bagfile_split_duration_s_{60};
  typedef std::map<TopicDetails, std::shared_ptr<MessageQueue>> buffers_t;
  typedef std::vector<std::pair<TopicDetails, std::shared_ptr<MessageQueue>>>
    selected_buffers_t;
  struct SnapshotEstimateResult
  {
    bool success{false};
    uint64_t requested_bytes{0};
    uint64_t selected_bytes{0};
    uint64_t message_count{0};
    int64_t actual_start_ns{0};
    int64_t newest_ns{0};
    bool truncated{false};
    std::string message;
  };
  buffers_t buffers_;
  // Protect the topic registry while runtime topics are added/removed.
  std::mutex buffers_lock_;
  // Only runtime-added topics may be removed through the configure service.
  std::map<std::string, TopicDetails> dynamic_topics_;
  // Locks recording_ and writing_ states.
  std::shared_mutex state_lock_;
  // True if new messages are being written to the internal buffer
  bool recording_;
  // True if currently writing buffers to a bag file
  bool writing_;
  rclcpp::Service<avg_msgs::srv::TriggerSnapshot>::SharedPtr
    trigger_snapshot_server_;
  rclcpp::Service<avg_msgs::srv::EstimateSnapshot>::SharedPtr
    estimate_snapshot_server_;
  rclcpp::Service<avg_msgs::srv::ConfigureSnapshotTopics>::SharedPtr
    configure_topics_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_server_;
  rclcpp::TimerBase::SharedPtr poll_topic_timer_;

  // HH_260921 - Automatic evidence capture. This package deliberately knows
  // nothing about lanelets, control or diagnostics semantics: a rule names a
  // topic and the operating states or severity that deserve a bag, so the
  // policy lives in configuration and the snapshotter keeps one responsibility.
  struct AutoTriggerRule
  {
    enum class Kind
    {
      kModuleState,
      kSystemStatus,
      kServiceState
    };

    std::string name;
    std::string topic;
    Kind kind{Kind::kModuleState};
    // Match the typed operating_state field, never the free-text message.
    // That message is a human-readable log line whose format may change.
    std::set<std::string> operating_states;
    // kSystemStatus only: restrict the per-module tests to these names.
    std::set<std::string> module_names;
    // Severity test is disabled while negative.
    int min_level{-1};
    // kSystemStatus only: fire on the aggregate system_ok flag.
    bool on_system_not_ok{false};
    // The condition must persist this long before a bag is written, so one
    // dropped heartbeat cannot spend the buffer. Leave it at 0 for a condition
    // that clears itself: such an event is counted, not waited out.
    double hold_s{0.0};
    // Fire on the Nth rising edge rather than the first. A condition whose
    // own recovery resolves it in under a second is a recurrence problem, and
    // recurrence is what a bag should capture.
    int min_occurrences{1};
    // Scope for that count: which topic says when the rule is live, and where
    // one counting episode ends. This is how "N times within one mission, and
    // only while it matters" is expressed without this package knowing what a
    // mission is.
    std::string scope_topic;
    Kind scope_kind{Kind::kServiceState};
    // Count only while the scope topic reports one of these. Empty means the
    // rule is live at all times.
    std::set<std::string> scope_active_states;
    // Clear the count on entry into one of these.
    std::set<std::string> scope_reset_states;
    // A fault that was never preceded by a healthy report is a cold start,
    // not a regression: boot ordering reaches ERROR before it reaches OK,
    // while a real fault can only follow an OK. Rules stay disarmed until the
    // watched topic has reported healthy once, which excludes startup by the
    // shape of the transition rather than by guessing how long boot takes.
    bool require_healthy_first{true};
    // The healthy report must itself hold this long, so modules flapping
    // through OK during boot ordering cannot arm the rule.
    double require_healthy_s{0.0};

    // Runtime state, guarded by auto_trigger_lock_.
    bool armed{true};
    bool matching{false};
    double matching_since_s{0.0};
    bool clear_since_valid{false};
    double clear_since_s{0.0};
    int occurrences{0};
    // Latched once the count is reached, so a contact that clears before the
    // next evaluation tick still gets its bag.
    bool fire_pending{false};
    std::string pending_detail;
    // The last scope state observed, so the count clears on every entry into
    // a reset state - including two different reset states in a row, which a
    // simple in-state flag would collapse into one.
    std::string last_scope_state;
    bool scope_state_valid{false};
    bool scope_active{true};
    std::string detail;
    rclcpp::SubscriptionBase::SharedPtr subscription;
    rclcpp::SubscriptionBase::SharedPtr scope_subscription;
  };

  using Kind_t = AutoTriggerRule::Kind;

  struct AutoTriggerCapture
  {
    std::string rule_name;
    std::string detail;
  };

  bool auto_trigger_enabled_{false};
  std::string auto_trigger_directory_;
  std::string auto_trigger_prefix_{"autosnapshot"};
  double auto_trigger_cooldown_s_{0.0};
  double auto_trigger_startup_grace_s_{60.0};
  double auto_trigger_lookback_s_{0.0};
  uint64_t auto_trigger_minimum_free_mb_{5120};
  double auto_trigger_minimum_free_ratio_{0.10};
  double auto_trigger_size_safety_factor_{1.30};
  std::vector<std::shared_ptr<AutoTriggerRule>> auto_trigger_rules_;
  // Guards rule runtime state, the pending queue and the cooldown clock.
  std::mutex auto_trigger_lock_;
  std::condition_variable auto_trigger_cv_;
  std::deque<AutoTriggerCapture> auto_trigger_pending_;
  bool auto_trigger_fired_{false};
  double auto_trigger_last_fire_s_{0.0};
  double auto_trigger_ready_after_s_{0.0};
  bool auto_trigger_shutdown_{false};
  // A bag write takes seconds to minutes. Keep it off the executor so the
  // trigger/estimate services stay answerable while a capture runs.
  std::thread auto_trigger_worker_;
  rclcpp::TimerBase::SharedPtr auto_trigger_timer_;

  // Convert parameter values into a SnapshotterOptions object
  void parseOptionsFromParams();
  // Replace individual topic limits with node defaults if they are
  // flagged for it (see SnapshotterTopicOptions)
  void fixTopicOptions(SnapshotterTopicOptions & options);
  // If file is "prefix" mode (doesn't end in .bag), append current datetime and .bag to end
  bool postfixFilename(std::string & file);
  /// Return current local datetime as a string such as 2018-05-22-14-28-51.
  // Used to generate bag filenames
  std::string timeAsStr();
  // Clear the internal buffers of all topics. Used when resuming after a pause to avoid time gaps
  void clear();
  // Subscribe to one of the topics, setting up the callback to add to the respective queue
  void subscribe(
    const TopicDetails & topic_details,
    std::shared_ptr<MessageQueue> queue);
  // Called on new message from any configured topic. Adds to queue for that topic
  void topicCb(
    std::shared_ptr<const rclcpp::SerializedMessage> msg,
    std::shared_ptr<MessageQueue> queue);
  // Service callback, write all of part of the internal buffers to a bag file
  // according to request parameters
  void triggerSnapshotCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const avg_msgs::srv::TriggerSnapshot::Request::SharedPtr req,
    avg_msgs::srv::TriggerSnapshot::Response::SharedPtr res
  );
  // Estimate a requested window and optionally trim it to the newest messages
  // that fit a serialized-byte budget.
  void estimateSnapshotCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const avg_msgs::srv::EstimateSnapshot::Request::SharedPtr req,
    avg_msgs::srv::EstimateSnapshot::Response::SharedPtr res
  );
  // Add/remove runtime-only subscriptions or return the current registry.
  void configureTopicsCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const avg_msgs::srv::ConfigureSnapshotTopics::Request::SharedPtr req,
    avg_msgs::srv::ConfigureSnapshotTopics::Response::SharedPtr res
  );
  // Service callback, enable or disable recording (storing new messages into queue).
  // Used to pause before writing
  void enableCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool_Response::SharedPtr res
  );
  // Set recording_ to false and do nessesary cleaning, CALLER MUST OBTAIN LOCK
  void pause();
  // Set recording_ to true and do nesessary cleaning, CALLER MUST OBTAIN LOCK.
  // `clear_buffers` is false when the write never began: a rejected snapshot
  // must not cost the operator the history it was meant to preserve.
  void resume(bool clear_buffers = true);
  // Poll master for new topics
  void pollTopics();
  selected_buffers_t selectBuffers(const std::vector<DetailsMsg> & requested_topics);
  SnapshotEstimateResult estimateBuffers(
    const selected_buffers_t & selected,
    const rclcpp::Time & start,
    const rclcpp::Time & stop,
    uint64_t max_bytes);
  bool hasMinimumDiskSpace(const std::string & filename, uint64_t minimum_free_bytes) const;
  // Write the parts of message_queue within the time constraints of req to the queue
  // If returns false, there was an error opening/writing the bag and an error message
  // was written to res.message
  bool writeTopic(
    rosbag2_cpp::Writer & bag_writer, MessageQueue & message_queue,
    const TopicDetails & topic_details,
    const avg_msgs::srv::TriggerSnapshot::Request::SharedPtr & req,
    const avg_msgs::srv::TriggerSnapshot::Response::SharedPtr & res,
    uint64_t & messages_written,
    uint64_t & bytes_written,
    uint64_t & bytes_since_space_check);
  // Shared write path for the trigger service and for automatic captures, so
  // both go through the same pause/estimate/disk-reserve guarantees.
  // `origin` only labels the offload log line, so an operator snapshot and an
  // automatic capture can be told apart in the transfer history.
  void writeSnapshot(
    const avg_msgs::srv::TriggerSnapshot::Request::SharedPtr & req,
    const avg_msgs::srv::TriggerSnapshot::Response::SharedPtr & res,
    const std::string & origin);

  // Read the auto-trigger rule table. Throws on a malformed rule so a broken
  // evidence-capture configuration fails at startup rather than silently
  // never firing.
  // HH_260921 - Offload a finished bag to shared storage. Both the trigger
  // service and an automatic capture end in writeSnapshot(), so hooking the
  // transfer there covers the operator UI and the auto-trigger alike.
  struct OffloadRequest
  {
    std::string local_path;
    std::string origin;
  };

  bool offload_enabled_{false};
  std::string offload_host_;
  std::string offload_user_;
  int offload_port_{22};
  std::string offload_remote_directory_;
  std::string offload_identity_file_;
  // False turns the move into a copy, leaving the local bag in place.
  bool offload_remove_local_{true};
  int offload_connect_timeout_s_{10};
  int offload_transfer_timeout_s_{1800};
  int offload_retries_{2};
  int offload_retry_delay_s_{30};
  std::mutex offload_lock_;
  std::condition_variable offload_cv_;
  std::deque<OffloadRequest> offload_pending_;
  bool offload_shutdown_{false};
  // Its own thread: a slow or stalled network transfer must not delay the
  // next automatic capture, and must never touch the buffer path.
  std::thread offload_worker_;

  void parseOffloadParams();
  void startOffload();
  void stopOffload();
  // Queue a finished bag. Safe to call from the service thread and from the
  // capture worker.
  void enqueueOffload(const std::string & local_path, const std::string & origin);
  void runOffloadWorker();
  // One transfer attempt. Returns false and fills `error` on any failure.
  bool offloadOnce(const OffloadRequest & request, std::string & error);
  // Run argv directly, without a shell, so a bag path can never be parsed as
  // a command. Returns the exit status, or -1 if it could not run, timed out
  // or was aborted.
  int runProcess(const std::vector<std::string> & argv, int timeout_s, std::string & error);
  std::vector<std::string> sshOptionArgs() const;

  void parseAutoTriggerParams();
  void startAutoTrigger();
  void stopAutoTrigger();
  void onAutoTriggerModuleState(
    const std::shared_ptr<AutoTriggerRule> & rule,
    const avg_msgs::msg::ModuleState & msg);
  void onAutoTriggerSystemStatus(
    const std::shared_ptr<AutoTriggerRule> & rule,
    const avg_msgs::msg::SystemStatus & msg);
  void onAutoTriggerServiceState(
    const std::shared_ptr<AutoTriggerRule> & rule,
    const avg_msgs::msg::AvgServiceState & msg);
  // Subscribe one rule endpoint. `scope` selects the counter-scope topic
  // instead of the condition topic.
  rclcpp::SubscriptionBase::SharedPtr subscribeAutoTriggerEndpoint(
    const std::shared_ptr<AutoTriggerRule> & rule, Kind_t kind, const std::string & topic,
    bool scope);
  // Track whether the rule is live, and clear its count when the scope topic
  // opens a new episode.
  void noteAutoTriggerScope(
    const std::shared_ptr<AutoTriggerRule> & rule, const std::string & scope_state);
  // Record a rule's latest verdict and keep its rising edge.
  void noteAutoTriggerMatch(
    const std::shared_ptr<AutoTriggerRule> & rule, bool matched, const std::string & detail);
  // Promote a held condition into a pending capture.
  void evaluateAutoTriggers();
  void runAutoTriggerWorker();
  void captureAutoTrigger(const AutoTriggerCapture & capture);
  // Mirror the operator UI's storage policy so an automatic bag leaves the
  // same filesystem reserve a manual one does.
  void autoTriggerStorageBudget(
    const std::string & probe_path, uint64_t & reserve_bytes, uint64_t & max_bytes) const;
  // Auto-trigger timing is monotonic on purpose: cooldown and startup grace
  // must not jump when the node runs under simulated or resynchronized time.
  static double steadySeconds();
};

// Configuration for SnapshotterClient
struct SnapshotterClientOptions
{
  SnapshotterClientOptions();
  enum Action
  {
    TRIGGER_WRITE,
    PAUSE,
    RESUME
  };
  // What to do when SnapshotterClient.run is called
  Action action_;
  // List of topics to write when action_ == TRIGGER_WRITE.
  // If empty, write all buffered topics.
  std::vector<TopicDetails> topics_;
  // Name of file to write to when action_ == TRIGGER_WRITE, relative to snapshot node.
  // If empty, use prefix
  std::string filename_;
  // Prefix of the name of file written to when action_ == TRIGGER_WRITE.
  std::string prefix_;
};

// Node used to call services which interface with the snapshotter node to trigger
// write, pause, and resume
class SnapshotterClient : public rclcpp::Node
{
public:
  explicit SnapshotterClient(const rclcpp::NodeOptions & options);

private:
  void setSnapshotterClientOptions(SnapshotterClientOptions const & opts);
};

}  // namespace camrod_snapshot

#endif  // CAMROD_SNAPSHOT__SNAPSHOTTER_HPP_
