#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// HH_260720 - Use generated CAMROD range/grid contracts and ROS geometry only for TF.
#include <avg_msgs/conversions.hpp>
#include <avg_msgs/msg/avg_bool.hpp>
#include <avg_msgs/msg/avg_string.hpp>
#include <avg_msgs/msg/avg_sensing_radar.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <avg_msgs/msg/avg_occupancy_grid.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/msg/avg_range.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "camrod_sensing/radar_dummy_contract.hpp"
#include "camrod_sensing/radar_self_echo_filter.hpp"
#include "camrod_sensing/route_lanelet_cost_filter.hpp"

namespace camrod::sensing
{

class RadarCostGridNode : public rclcpp::Node
{
public:
  // HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.

  // Implements `RadarCostGridNode` behavior.
  RadarCostGridNode()
  : Node("radar_cost_grid")
  {
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    startup_only.description =
      "Startup-only radar cost-grid setting; restart the node to apply";

    // HH_260729 - Give every declared setting the same read-only descriptor.
    // The parameter service and `ros2 param describe` must agree that YAML
    // edits require a node restart.
    const auto declare_startup =
      [this, &startup_only](const std::string & name, const auto & default_value) {
        return declare_parameter(name, default_value, startup_only);
      };

    output_topic_ = declare_startup(
      "output_topic", std::string{"/sensing/cost_grid/radar"});
    base_frame_id_ = declare_startup("base_frame_id", std::string{"robot_base_link"});
    output_frame_id_ = declare_startup("output_frame_id", std::string{"map"});
    // HH_260729 - Numeric no-target ranges already sit above max_range. The
    // explicit dummy heartbeat is a second, independent barrier so disabled
    // global or per-channel radar inputs can never paint obstacle costs.
    dummy_active_topic_ = declare_startup(
      "dummy_active_topic", std::string{"/sensing/radar/dummy_active"});
    dummy_active_timeout_s_ = declare_startup("dummy_active_timeout_s", 1.0);
    resolution_ = declare_startup("resolution", 0.10);
    width_ = declare_startup("width", 120);
    height_ = declare_startup("height", 120);
    origin_x_ = declare_startup("origin_x", -6.0);
    origin_y_ = declare_startup("origin_y", -6.0);
    free_value_ = declare_startup("free_value", 0);
    unknown_value_ = declare_startup("unknown_value", -1);
    min_cost_ = declare_startup("min_cost", 85);
    max_cost_ = declare_startup("max_cost", 100);
    cost_near_distance_m_ = declare_startup("cost_near_distance_m", 0.3);
    // HH_260422 - Radar is near-field only; the per-sensor Range max also
    // rejects detections before they reach this cost-mapping stage.
    cost_far_distance_m_ = declare_startup("cost_far_distance_m", 2.0);
    obstacle_radius_m_ = declare_startup("obstacle_radius_m", 0.30);
    // HH_260422 - Keep near-field side/rear hits outside the center clear disk.
    ego_clear_radius_m_ = declare_startup("ego_clear_radius_m", 0.50);

    // HH_260729 - Exclude only explicitly measured, narrow persistent-return
    // intervals. Each entry names its radar directly as SENSOR:min_m:max_m.
    fixed_return_filter_enable_ =
      declare_startup("fixed_return_filter_enable", true);
    fixed_return_band_specs_ = declare_startup(
      "fixed_return_bands", std::vector<std::string>{});
    // HH_260728 - Hardware restarts can select a different stable body/multipath
    // return. Learn one dominant tight interval per sensor while disengaged,
    // then freeze the result for the complete node lifetime.
    startup_return_learning_enable_ =
      declare_startup("startup_return_learning_enable", true);
    startup_return_learning_duration_s_ =
      declare_startup("startup_return_learning_duration_s", 8.0);
    startup_return_first_sample_timeout_s_ =
      declare_startup("startup_return_first_sample_timeout_s", 15.0);
    startup_return_min_samples_ =
      declare_startup("startup_return_min_samples", 15);
    startup_return_cluster_gap_m_ =
      declare_startup("startup_return_cluster_gap_m", 0.020);
    startup_return_min_cluster_fraction_ =
      declare_startup("startup_return_min_cluster_fraction", 0.50);
    startup_return_min_half_width_m_ =
      declare_startup("startup_return_min_half_width_m", 0.012);
    startup_return_max_half_width_m_ =
      declare_startup("startup_return_max_half_width_m", 0.030);
    startup_return_margin_m_ =
      declare_startup("startup_return_margin_m", 0.005);
    startup_return_default_max_range_m_ =
      declare_startup("startup_return_default_max_range_m", 0.30);
    startup_return_max_ranges_m_ = declare_startup(
      "startup_return_max_ranges_m", std::vector<double>{});
    startup_return_authorization_topic_ = declare_startup(
      "startup_return_authorization_topic", std::string{"/control/planning_engaged"});

    max_message_age_s_ = declare_startup("max_message_age_s", 0.35);
    publish_rate_hz_ = declare_startup("publish_rate_hz", 10.0);
    radar_status_topic_ = declare_startup(
      "radar_status_topic", std::string{"/sensing/radar/status"});
    publish_radar_status_ = declare_startup("publish_radar_status", false);
    obstacle_evidence_topic_ = declare_startup(
      "obstacle_evidence_topic", std::string{"/sensing/radar/obstacle_evidence"});
    obstacle_evidence_warn_interval_s_ = declare_startup(
      "obstacle_evidence_warn_interval_s", 1.0);
    // HH_260720 - Apply the same active-route obstacle corridor to radar and
    // LiDAR so the safety gate and merged planning grid cannot disagree.
    route_lanelet_filter_enable_ =
      declare_startup("route_lanelet_filter_enable", true);
    route_lanelet_mask_topic_ = declare_startup(
      "route_lanelet_mask_topic", std::string{"/map/cost_grid/route_lanelet_mask"});
    route_lanelet_margin_m_ = declare_startup("route_lanelet_margin_m", 0.35);
    route_lanelet_allowed_max_cost_ =
      declare_startup("route_lanelet_allowed_max_cost", 50);
    route_lanelet_mask_max_age_s_ =
      declare_startup("route_lanelet_mask_max_age_s", 2.5);
    route_lanelet_filter_fail_open_when_robot_outside_ = declare_startup(
      "route_lanelet_filter_fail_open_when_robot_outside", true);
    input_topics_ = declare_startup(
      "input_topics",
      // HH_260623 - Use the latest 7-channel radar topic set from todo/camrod_sensing.
      std::vector<std::string>{
        "/sensing/radar/front1/range",
        "/sensing/radar/front2/range",
        "/sensing/radar/left1/range",
        "/sensing/radar/left2/range",
        "/sensing/radar/right1/range",
        "/sensing/radar/right2/range",
        "/sensing/radar/rear/range"});
    if (!std::isfinite(dummy_active_timeout_s_) ||
      dummy_active_timeout_s_ <= 0.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "invalid dummy_active_timeout_s %.3f; using fail-visible default 1.0 s",
        dummy_active_timeout_s_);
      dummy_active_timeout_s_ = 1.0;
    }

    // HH_260729 - Every cost-grid setting is cached into validated runtime
    // state during construction. Reject later parameter writes explicitly so
    // `ros2 param set` cannot report success while filtering stays unchanged.
    startup_only_param_callback_handle_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : params) {
          if (parameter.get_name() == "use_sim_time") {
            continue;
          }
          result.successful = false;
          result.reason =
            "radar cost-grid parameters are startup-only; edit YAML and restart the node";
          break;
        }
        return result;
      });

    configureFixedReturnFilter();
    startup_return_samples_.resize(input_topics_.size());
    startup_return_first_sample_elapsed_s_.assign(
      input_topics_.size(), std::numeric_limits<double>::quiet_NaN());
    startup_return_sensor_finalized_.assign(input_topics_.size(), false);
    startup_return_started_at_ = std::chrono::steady_clock::now();
    if (startup_return_learning_enable_) {
      RCLCPP_INFO(
        get_logger(),
        "startup radar return learning gives each sensor %.1f s after its first valid "
        "sample (first-sample timeout %.1f s); awaiting transient authorization state",
        startup_return_learning_duration_s_,
        startup_return_first_sample_timeout_s_);
    } else {
      startup_return_learning_finalized_ = true;
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // HH_260720 - Publish radar cost data on the generated CAMROD grid contract.
    pub_grid_ = create_publisher<avg_msgs::msg::AvgOccupancyGrid>(
      output_topic_, rclcpp::QoS(1).transient_local().reliable());
    avg_radar_pub_ = create_publisher<avg_msgs::msg::AvgSensingRadar>(radar_status_topic_, rclcpp::QoS(10));
    if (!obstacle_evidence_topic_.empty()) {
      obstacle_evidence_pub_ = create_publisher<avg_msgs::msg::AvgString>(
        obstacle_evidence_topic_, rclcpp::QoS(10));
    } else {
      RCLCPP_WARN(
        get_logger(),
        "radar obstacle evidence publisher disabled because obstacle_evidence_topic is empty");
    }
    // HH_260720 - Receive the latest active-route mask across startup order.
    route_lanelet_mask_sub_ = create_subscription<avg_msgs::msg::AvgOccupancyGrid>(
      route_lanelet_mask_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const avg_msgs::msg::AvgOccupancyGrid::ConstSharedPtr msg) {
        onRouteLaneletMask(msg);
      });
    const auto dummy_state_qos = rclcpp::QoS(1).reliable().transient_local();
    if (!dummy_active_topic_.empty()) {
      global_dummy_active_sub_ = create_subscription<avg_msgs::msg::AvgBool>(
        dummy_active_topic_, dummy_state_qos,
        [this](const avg_msgs::msg::AvgBool::ConstSharedPtr msg) {
          onDummyState(global_dummy_state_, msg);
        });
    } else {
      RCLCPP_WARN(
        get_logger(),
        "radar global dummy-state barrier disabled because dummy_active_topic is empty");
    }
    if (startup_return_learning_enable_ &&
      !startup_return_authorization_topic_.empty())
    {
      startup_return_authorization_sub_ = create_subscription<avg_msgs::msg::AvgBool>(
        startup_return_authorization_topic_,
        rclcpp::QoS(1).reliable().transient_local(),
        [this](const avg_msgs::msg::AvgBool::ConstSharedPtr msg) {
          if (!msg) {
            return;
          }
          startup_return_authorization_received_ = true;
          startup_return_motion_authorized_ = msg->data;
          // HH_260728 - The safety gate publishes manual-or-mission engage as
          // transient local state. A restarted cost-grid node therefore
          // cancels learning even while effective command output is cost-held.
          if (!msg->data || startup_return_learning_finalized_) {
            return;
          }
          startup_return_learning_finalized_ = true;
          startup_return_samples_.clear();
          RCLCPP_WARN(
            get_logger(),
            "startup radar return learning cancelled because planning authorization is active");
        });
    } else if (startup_return_learning_enable_) {
      RCLCPP_WARN(
        get_logger(),
        "startup radar return learning has no authorization topic; no samples will be learned");
    }

    samples_.resize(input_topics_.size());
    channel_dummy_states_.resize(input_topics_.size());
    for (std::size_t i = 0; i < input_topics_.size(); ++i) {
      subs_.push_back(create_subscription<avg_msgs::msg::AvgRange>(
        input_topics_[i], rclcpp::SensorDataQoS(),
        [this, i](avg_msgs::msg::AvgRange::ConstSharedPtr msg) { onRange(i, msg); }));
      try {
        const auto channel_dummy_topic =
          camrod_sensing::radar_dummy_active_topic(input_topics_[i]);
        channel_dummy_active_subs_.push_back(
          create_subscription<avg_msgs::msg::AvgBool>(
            channel_dummy_topic, dummy_state_qos,
            [this, i](const avg_msgs::msg::AvgBool::ConstSharedPtr msg) {
              onDummyState(channel_dummy_states_[i], msg);
            }));
      } catch (const std::invalid_argument & error) {
        RCLCPP_ERROR(
          get_logger(),
          "cannot derive per-channel dummy-state topic for %s: %s",
          input_topics_[i].c_str(), error.what());
        channel_dummy_active_subs_.push_back(nullptr);
      }
    }

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 10.0;
    }
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RadarCostGridNode::publishGrid, this));
  }

private:
  struct DummyState
  {
    rclcpp::Time recv_time{0, 0, RCL_ROS_TIME};
    bool received{false};
    bool active{false};
  };

  struct RangeSample
  {
    avg_msgs::msg::AvgRange msg;
    rclcpp::Time recv_time{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  void onDummyState(
    DummyState & state,
    const avg_msgs::msg::AvgBool::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    state.received = true;
    state.active = msg->data;
    state.recv_time = now();
  }

  bool dummyStateSuppressesCost(
    const DummyState & state,
    const rclcpp::Time & now_time) const
  {
    const double age_s =
      state.received ? (now_time - state.recv_time).seconds() : -1.0;
    return camrod_sensing::radar_dummy_state_suppresses_cost(
      state.received, state.active, age_s, dummy_active_timeout_s_);
  }

  // HH_260729 - Preserve the channel identity only for diagnostic reporting.
  // Cost-grid painting and safety thresholds remain unchanged.
  struct ActiveHitEvidence
  {
    std::string sensor;
    std::string frame_id;
    double range_m{0.0};
    double output_x{0.0};
    double output_y{0.0};
    int cost{0};
    std::vector<std::size_t> painted_cells;
  };

  // Handles the `onRange` callback.
  void onRange(std::size_t idx, const avg_msgs::msg::AvgRange::ConstSharedPtr msg)
  {
    if (!msg || idx >= samples_.size()) {
      return;
    }
    samples_[idx].msg = *msg;
    samples_[idx].recv_time = now();
    samples_[idx].valid = true;
    if (startup_return_learning_enable_ &&
      !startup_return_learning_finalized_ &&
      startup_return_authorization_received_ &&
      !startup_return_motion_authorized_ &&
      idx < startup_return_sensor_finalized_.size() &&
      !startup_return_sensor_finalized_[idx] &&
      std::isfinite(msg->range) && std::isfinite(msg->min_range) &&
      std::isfinite(msg->max_range) &&
      msg->range >= msg->min_range && msg->range <= msg->max_range &&
      msg->range <= startupReturnMaxRangeForIndex(idx) &&
      startup_return_samples_[idx].size() < 1000U)
    {
      if (!std::isfinite(startup_return_first_sample_elapsed_s_[idx])) {
        startup_return_first_sample_elapsed_s_[idx] =
          startupReturnElapsedSeconds();
        RCLCPP_INFO(
          get_logger(), "startup fixed-return collection started for %s",
          input_topics_[idx].c_str());
      }
      startup_return_samples_[idx].push_back(msg->range);
    }
  }

  // HH_260720 - Cache route-mask validity once per map update.
  void onRouteLaneletMask(const avg_msgs::msg::AvgOccupancyGrid::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    route_lanelet_mask_ = msg;
    route_lanelet_mask_receive_time_ = now();
    route_lanelet_mask_has_allowed_cells_ =
      route_lanelet_cost_filter::hasAllowedCell(*msg, route_lanelet_allowed_max_cost_);
  }

  bool shouldApplyRouteLaneletFilter(
    const geometry_msgs::msg::PointStamped & base_in_output, const rclcpp::Time & now_time)
  {
    if (!route_lanelet_filter_enable_) {
      return false;
    }
    if (!route_lanelet_mask_ || !route_lanelet_mask_has_allowed_cells_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "route lanelet obstacle filter waiting for a valid active-route mask; passing costs through");
      return false;
    }
    if (route_lanelet_mask_max_age_s_ > 0.0 &&
      (now_time - route_lanelet_mask_receive_time_).seconds() > route_lanelet_mask_max_age_s_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "route lanelet obstacle mask is stale; passing costs through");
      return false;
    }
    if (route_lanelet_mask_->header.frame_id != output_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "route lanelet obstacle mask frame mismatch (%s != %s); passing costs through",
        route_lanelet_mask_->header.frame_id.c_str(), output_frame_id_.c_str());
      return false;
    }
    if (route_lanelet_filter_fail_open_when_robot_outside_ &&
      !route_lanelet_cost_filter::isWorldPointAllowed(
        *route_lanelet_mask_, base_in_output.point.x, base_in_output.point.y,
        route_lanelet_margin_m_, route_lanelet_allowed_max_cost_))
    {
      // HH_260720 - Preserve unfiltered obstacle protection during deliberate
      // off-route campsite and parking maneuvers.
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "robot is outside the active route lanelet corridor; passing obstacle costs through");
      return false;
    }
    return true;
  }

  // Implements `mapDistanceToCost` behavior.
  int mapDistanceToCost(const double distance_m) const
  {
    const double far_distance_m =
      std::max(cost_near_distance_m_ + 1e-3, cost_far_distance_m_);
    if (distance_m <= cost_near_distance_m_) {
      return max_cost_;
    }
    if (distance_m >= far_distance_m) {
      return min_cost_;
    }
    const double norm =
      (distance_m - cost_near_distance_m_) /
      (far_distance_m - cost_near_distance_m_);
    const double inv = 1.0 - std::clamp(norm, 0.0, 1.0);
    return static_cast<int>(std::round(
      static_cast<double>(min_cost_) +
      inv * static_cast<double>(max_cost_ - min_cost_)));
  }

  // Implements `markDisk` behavior.
  void markDisk(
    avg_msgs::msg::AvgOccupancyGrid & grid,
    const double grid_origin_x,
    const double grid_origin_y,
    const double x,
    const double y,
    const int value,
    std::vector<std::size_t> * painted_cells = nullptr)
  {
    const int cx = static_cast<int>(std::floor((x - grid_origin_x) / resolution_));
    const int cy = static_cast<int>(std::floor((y - grid_origin_y) / resolution_));
    const int radius_cells =
      static_cast<int>(std::ceil(std::max(0.0, obstacle_radius_m_) / resolution_));

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int gx = cx + dx;
        const int gy = cy + dy;
        if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_) {
          continue;
        }
        const std::size_t idx = static_cast<std::size_t>(gy * width_ + gx);
        if (painted_cells != nullptr) {
          painted_cells->push_back(idx);
        }
        if (grid.data[idx] < 0) {
          grid.data[idx] = static_cast<int8_t>(value);
        } else {
          grid.data[idx] = static_cast<int8_t>(std::max<int>(grid.data[idx], value));
        }
      }
    }
  }

  // Implements `clearDisk` behavior.
  void clearDisk(
    avg_msgs::msg::AvgOccupancyGrid & grid,
    const double grid_origin_x,
    const double grid_origin_y,
    const double x,
    const double y)
  {
    const int cx = static_cast<int>(std::floor((x - grid_origin_x) / resolution_));
    const int cy = static_cast<int>(std::floor((y - grid_origin_y) / resolution_));
    const int radius_cells =
      static_cast<int>(std::ceil(std::max(0.0, ego_clear_radius_m_) / resolution_));

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int gx = cx + dx;
        const int gy = cy + dy;
        if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_) {
          continue;
        }
        const std::size_t idx = static_cast<std::size_t>(gy * width_ + gx);
        grid.data[idx] = static_cast<int8_t>(free_value_);
      }
    }
  }

  double startupReturnMaxRangeForIndex(const std::size_t idx) const
  {
    if (idx < startup_return_max_ranges_m_.size() &&
      std::isfinite(startup_return_max_ranges_m_[idx]) &&
      startup_return_max_ranges_m_[idx] > 0.0)
    {
      return startup_return_max_ranges_m_[idx];
    }
    return std::max(0.0, startup_return_default_max_range_m_);
  }

  void configureFixedReturnFilter()
  {
    fixed_return_exclusion_bands_.clear();
    if (!fixed_return_filter_enable_) {
      RCLCPP_INFO(get_logger(), "radar fixed-return exclusion filter disabled");
      return;
    }

    std::string error;
    if (!radar_self_echo_filter::buildBandsFromSpecs(
        fixed_return_band_specs_, input_topics_, fixed_return_exclusion_bands_, error))
    {
      // HH_260729 - Fail safe: a malformed profile disables both fixed
      // exclusion and startup learning, leaving every valid range available
      // instead of applying a partial or boot-local blind zone.
      RCLCPP_ERROR(
        get_logger(), "invalid radar fixed-return profile; filter disabled: %s", error.c_str());
      fixed_return_exclusion_bands_.clear();
      fixed_return_filter_enable_ = false;
      startup_return_learning_enable_ = false;
      startup_return_learning_finalized_ = true;
      return;
    }

    RCLCPP_INFO(
      get_logger(), "radar fixed-return exclusion filter active with %zu measured bands",
      fixed_return_exclusion_bands_.size());
    // HH_260729 - Emit each resolved sensor and interval at startup so field
    // diagnosis does not require translating an index into input_topics.
    for (const auto & band : fixed_return_exclusion_bands_) {
      std::string sensor_label;
      std::string label_error;
      if (band.sensor_index >= input_topics_.size() ||
        !radar_self_echo_filter::sensorLabelFromTopic(
          input_topics_[band.sensor_index], sensor_label, label_error))
      {
        sensor_label = "INDEX_" + std::to_string(band.sensor_index);
      }
      RCLCPP_INFO(
        get_logger(), "radar fixed-return band %s:%.3f..%.3f m",
        sensor_label.c_str(), band.center_m - band.half_width_m,
        band.center_m + band.half_width_m);
    }
  }

  void maybeFinalizeStartupReturnLearning()
  {
    // HH_260728 - Freeze startup-only learning after its bounded disengaged
    // interval; no runtime observation can widen or move these bands later.
    if (!startup_return_learning_enable_ ||
      startup_return_learning_finalized_)
    {
      return;
    }
    const double elapsed_s = startupReturnElapsedSeconds();
    const auto minimum_samples = static_cast<std::size_t>(
      std::max(1, startup_return_min_samples_));
    for (std::size_t i = 0; i < startup_return_samples_.size(); ++i) {
      const auto action = radar_self_echo_filter::startupCalibrationAction(
        startup_return_sensor_finalized_[i],
        startup_return_first_sample_elapsed_s_[i],
        elapsed_s,
        startup_return_learning_duration_s_,
        startup_return_first_sample_timeout_s_);
      if (action == radar_self_echo_filter::StartupCalibrationAction::kWait ||
        action == radar_self_echo_filter::StartupCalibrationAction::kDone)
      {
        continue;
      }
      if (action == radar_self_echo_filter::StartupCalibrationAction::kRejectNoTimelySample) {
        startup_return_sensor_finalized_[i] = true;
        RCLCPP_WARN(
          get_logger(),
          "startup return learning rejected for %s: no valid authorized sample before %.1f s",
          input_topics_[i].c_str(), startup_return_first_sample_timeout_s_);
        continue;
      }

      radar_self_echo_filter::Band learned;
      std::string error;
      if (!radar_self_echo_filter::learnDominantBand(
          i, startup_return_samples_[i], minimum_samples,
          startup_return_cluster_gap_m_, startup_return_min_cluster_fraction_,
          startup_return_min_half_width_m_, startup_return_max_half_width_m_,
          startup_return_margin_m_, learned, error))
      {
        RCLCPP_WARN(
          get_logger(), "startup return learning rejected for %s: %s (%zu samples)",
          input_topics_[i].c_str(), error.c_str(), startup_return_samples_[i].size());
      } else {
        fixed_return_exclusion_bands_.push_back(learned);
        ++startup_return_learned_count_;
        std::string sensor_label;
        std::string label_error;
        if (!radar_self_echo_filter::sensorLabelFromTopic(
            input_topics_[i], sensor_label, label_error))
        {
          sensor_label = input_topics_[i];
        }
        RCLCPP_INFO(
          get_logger(),
          "startup learned return %s:%.3f..%.3f m (%zu samples)",
          sensor_label.c_str(), learned.center_m - learned.half_width_m,
          learned.center_m + learned.half_width_m,
          startup_return_samples_[i].size());
      }
      startup_return_sensor_finalized_[i] = true;
      startup_return_samples_[i].clear();
    }

    if (std::all_of(
        startup_return_sensor_finalized_.begin(),
        startup_return_sensor_finalized_.end(),
        [](const bool finalized) { return finalized; }))
    {
      startup_return_learning_finalized_ = true;
      startup_return_samples_.clear();
      RCLCPP_INFO(
        get_logger(), "startup radar return learning frozen with %zu learned bands",
        startup_return_learned_count_);
    }
  }

  double startupReturnElapsedSeconds() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - startup_return_started_at_).count();
  }

  // Implements `transformHitToOutput` behavior.
  bool transformHitToOutput(
    const avg_msgs::msg::AvgRange & msg,
    geometry_msgs::msg::PointStamped & hit_output)
  {
    if (!std::isfinite(msg.range)) {
      return false;
    }
    if (msg.range < msg.min_range || msg.range > msg.max_range) {
      return false;
    }
    if (msg.header.frame_id.empty()) {
      return false;
    }

    geometry_msgs::msg::PointStamped hit_sensor;
    // HH_260720 - Convert the generated range header only for the tf2 ROS point.
    hit_sensor.header = avg_msgs::conversions::toRos(msg.header);
    if (hit_sensor.header.stamp.sec == 0 && hit_sensor.header.stamp.nanosec == 0) {
      hit_sensor.header.stamp = now();
    }
    hit_sensor.point.x = msg.range;
    hit_sensor.point.y = 0.0;
    hit_sensor.point.z = 0.0;

    try {
      hit_output = tf_buffer_->transform(
        hit_sensor, output_frame_id_, tf2::durationFromSec(0.05));
      return true;
    } catch (const tf2::TransformException &) {
      // HH_260315-00:00 Fallback to latest TF to avoid transient "future
      // extrapolation" drops that make marker/grid updates look unstable.
      hit_sensor.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      try {
        hit_output = tf_buffer_->transform(
          hit_sensor, output_frame_id_, tf2::durationFromSec(0.05));
        return true;
      } catch (const tf2::TransformException & ex_latest) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "radar_cost_grid TF transform failed (%s -> %s): %s",
          hit_sensor.header.frame_id.c_str(), output_frame_id_.c_str(), ex_latest.what());
        return false;
      }
    }
  }

  std::string sensorLabelForIndex(const std::size_t idx) const
  {
    if (idx >= input_topics_.size()) {
      return "INDEX_" + std::to_string(idx);
    }
    std::string label;
    std::string error;
    if (!radar_self_echo_filter::sensorLabelFromTopic(input_topics_[idx], label, error)) {
      return "INDEX_" + std::to_string(idx);
    }
    return label;
  }

  // HH_260729 - Route clipping happens after all radar disks are painted.
  // Report a candidate only when at least one cell painted by that exact hit
  // remains at its cost in the final published grid.
  bool evidenceRemainsInGrid(
    const ActiveHitEvidence & evidence,
    const avg_msgs::msg::AvgOccupancyGrid & grid) const
  {
    return std::any_of(
      evidence.painted_cells.begin(), evidence.painted_cells.end(),
      [&grid, &evidence](const std::size_t index) {
        return index < grid.data.size() &&
               static_cast<int>(grid.data[index]) >= evidence.cost;
      });
  }

  std::string formatObstacleEvidence(
    const std::vector<ActiveHitEvidence> & active_hits) const
  {
    if (active_hits.empty()) {
      return "clear";
    }

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << "active";
    for (const auto & hit : active_hits) {
      stream
        << " SENSOR=" << hit.sensor
        << " frame_id=" << hit.frame_id
        << " range_m=" << hit.range_m
        << " output_frame=" << output_frame_id_
        << " x=" << hit.output_x
        << " y=" << hit.output_y
        << " cost=" << hit.cost
        << ";";
    }
    return stream.str();
  }

  void publishObstacleEvidence(
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    const std::vector<ActiveHitEvidence> & candidates)
  {
    std::vector<ActiveHitEvidence> active_hits;
    active_hits.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      if (evidenceRemainsInGrid(candidate, grid)) {
        active_hits.push_back(candidate);
      }
    }

    const std::string evidence_text = formatObstacleEvidence(active_hits);
    if (obstacle_evidence_pub_) {
      avg_msgs::msg::AvgString message;
      message.data = evidence_text;
      obstacle_evidence_pub_->publish(message);
    }

    // HH_260729 - Keep active-hit warnings readable under a persistent return.
    // Clear, missing, stale, disabled, invalid, and filtered channels never
    // enter active_hits and therefore never produce this warning.
    if (active_hits.empty() || obstacle_evidence_warn_interval_s_ <= 0.0) {
      return;
    }
    const auto now_steady = std::chrono::steady_clock::now();
    const bool interval_elapsed =
      !obstacle_evidence_warned_ ||
      std::chrono::duration<double>(
        now_steady - last_obstacle_evidence_warn_at_).count() >=
      obstacle_evidence_warn_interval_s_;
    if (interval_elapsed) {
      RCLCPP_WARN(get_logger(), "radar obstacle evidence: %s", evidence_text.c_str());
      last_obstacle_evidence_warn_at_ = now_steady;
      obstacle_evidence_warned_ = true;
    }
  }

  // Publishes `Grid` output.
  void publishGrid()
  {
    maybeFinalizeStartupReturnLearning();

    geometry_msgs::msg::PointStamped base_origin;
    // HH_260315-00:00 Anchor rolling grid with latest available TF.
    base_origin.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    base_origin.header.frame_id = base_frame_id_;
    base_origin.point.x = 0.0;
    base_origin.point.y = 0.0;
    base_origin.point.z = 0.0;

    geometry_msgs::msg::PointStamped base_in_output;
    try {
      base_in_output = tf_buffer_->transform(
        base_origin, output_frame_id_, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "radar_cost_grid failed to locate %s in %s: %s",
        base_frame_id_.c_str(), output_frame_id_.c_str(), ex.what());
      return;
    }

    const double grid_origin_x = base_in_output.point.x + origin_x_;
    const double grid_origin_y = base_in_output.point.y + origin_y_;

    avg_msgs::msg::AvgOccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = output_frame_id_;
    grid.info.map_load_time = grid.header.stamp;
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<uint32_t>(width_);
    grid.info.height = static_cast<uint32_t>(height_);
    grid.info.origin.position.x = grid_origin_x;
    grid.info.origin.position.y = grid_origin_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    const int initial_value = (unknown_value_ >= -1 && unknown_value_ <= 100) ?
      unknown_value_ : free_value_;
    grid.data.assign(static_cast<std::size_t>(width_ * height_), static_cast<int8_t>(initial_value));

    // HH_260703 - Clear the ego footprint before marking live radar hits so
    // valid near-field side/rear detections can override the self-clear disk.
    if (ego_clear_radius_m_ > 0.0) {
      clearDisk(
        grid, grid_origin_x, grid_origin_y,
        base_in_output.point.x, base_in_output.point.y);
    }

    const auto now_time = now();
    std::vector<ActiveHitEvidence> evidence_candidates;
    evidence_candidates.reserve(samples_.size());
    std::vector<std::string> dummy_suppressed_sensors;
    dummy_suppressed_sensors.reserve(samples_.size());
    const bool global_dummy_active =
      dummyStateSuppressesCost(global_dummy_state_, now_time);
    for (std::size_t i = 0; i < samples_.size(); ++i) {
      const bool channel_dummy_active =
        i < channel_dummy_states_.size() &&
        dummyStateSuppressesCost(channel_dummy_states_[i], now_time);
      if (global_dummy_active || channel_dummy_active) {
        dummy_suppressed_sensors.push_back(sensorLabelForIndex(i));
        continue;
      }
      const auto & sample = samples_[i];
      if (!sample.valid) {
        continue;
      }
      if ((now_time - sample.recv_time).seconds() > max_message_age_s_) {
        continue;
      }
      if (fixed_return_filter_enable_ &&
        radar_self_echo_filter::matches(
          i, sample.msg.range, fixed_return_exclusion_bands_))
      {
        continue;
      }

      geometry_msgs::msg::PointStamped hit_output;
      if (!transformHitToOutput(sample.msg, hit_output)) {
        continue;
      }
      // HH_260630: SEN0592 risk should follow sensor-relative range, not
      // robot-base distance. Front/side/rear mounts are offset from base_link,
      // so base-distance scaling made close hand/obstacle hits fall below the
      // cmd_vel gate threshold.
      const int value = mapDistanceToCost(sample.msg.range);
      ActiveHitEvidence evidence;
      evidence.sensor = sensorLabelForIndex(i);
      evidence.frame_id = sample.msg.header.frame_id;
      evidence.range_m = sample.msg.range;
      evidence.output_x = hit_output.point.x;
      evidence.output_y = hit_output.point.y;
      evidence.cost = value;
      markDisk(
        grid, grid_origin_x, grid_origin_y, hit_output.point.x, hit_output.point.y, value,
        &evidence.painted_cells);
      if (!evidence.painted_cells.empty()) {
        evidence_candidates.push_back(std::move(evidence));
      }
    }
    if (!dummy_suppressed_sensors.empty()) {
      std::ostringstream suppressed;
      for (std::size_t i = 0; i < dummy_suppressed_sensors.size(); ++i) {
        if (i > 0U) {
          suppressed << ",";
        }
        suppressed << dummy_suppressed_sensors[i];
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "radar dummy-state barrier suppressing obstacle costs for %s",
        suppressed.str().c_str());
    }

    // HH_260720 - Clip fully-painted radar disks at the route lanelet margin,
    // preventing a hit in an adjacent lane from entering the driven corridor.
    // HH_260729 - An empty/dummy radar grid needs no route clipping. Avoid the
    // misleading "waiting for active-route mask" warning when radar hardware
    // is intentionally disabled and there is no obstacle cost to filter.
    if (!evidence_candidates.empty() &&
      shouldApplyRouteLaneletFilter(base_in_output, now_time))
    {
      const auto removed = route_lanelet_cost_filter::removeCostsOutsideRouteLanelets(
        grid, *route_lanelet_mask_, route_lanelet_margin_m_,
        route_lanelet_allowed_max_cost_, free_value_);
      if (removed > 0U) {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "removed %zu radar cost cells outside active route lanelets + %.2f m margin",
          removed, route_lanelet_margin_m_);
      }
    }

    publishObstacleEvidence(grid, evidence_candidates);
    pub_grid_->publish(grid);
    publishAvgRadar(grid);
  }

  // Implements `assignRangeByTopic` behavior.
  void assignRangeByTopic(avg_msgs::msg::AvgSensingRadar & avg_msg, std::size_t idx, const avg_msgs::msg::AvgRange & msg)
  {
    if (idx >= input_topics_.size()) {
      return;
    }
    const auto & topic = input_topics_[idx];
    // HH_260623 - Publish front1/front2 separately; merged front output was removed.
    if (topic.find("front1") != std::string::npos) {
      avg_msg.front1 = msg;
    } else if (topic.find("front2") != std::string::npos) {
      avg_msg.front2 = msg;
    } else if (topic.find("right1") != std::string::npos) {
      avg_msg.right1 = msg;
    } else if (topic.find("right2") != std::string::npos) {
      avg_msg.right2 = msg;
    } else if (topic.find("left1") != std::string::npos) {
      avg_msg.left1 = msg;
    } else if (topic.find("left2") != std::string::npos) {
      avg_msg.left2 = msg;
    } else if (topic.find("rear") != std::string::npos) {
      avg_msg.rear = msg;
    }
  }

  // Publishes `AvgRadar` output.
  void publishAvgRadar(const avg_msgs::msg::AvgOccupancyGrid & grid)
  {
    if (!publish_radar_status_ || !avg_radar_pub_) {
      return;
    }
    avg_msgs::msg::AvgSensingRadar avg_msg;
    // HH_260720 - Bundle the already-generated CAMROD grid without reconversion.
    avg_msg.near_cost_grid = grid;
    for (std::size_t i = 0; i < samples_.size(); ++i) {
      if (!samples_[i].valid) {
        continue;
      }
      assignRangeByTopic(avg_msg, i, samples_[i].msg);
    }
    avg_radar_pub_->publish(avg_msg);
  }

  std::string output_topic_;
  std::string radar_status_topic_;
  std::string obstacle_evidence_topic_;
  std::string base_frame_id_;
  std::string output_frame_id_;
  std::string dummy_active_topic_{"/sensing/radar/dummy_active"};
  double dummy_active_timeout_s_{1.0};
  double resolution_{0.10};
  int width_{120};
  int height_{120};
  double origin_x_{-6.0};
  double origin_y_{-6.0};
  int free_value_{0};
  int unknown_value_{0};
  int min_cost_{85};
  int max_cost_{100};
  double cost_near_distance_m_{0.3};
  bool fixed_return_filter_enable_{true};
  std::vector<std::string> fixed_return_band_specs_;
  std::vector<radar_self_echo_filter::Band> fixed_return_exclusion_bands_;
  bool startup_return_learning_enable_{true};
  double startup_return_learning_duration_s_{8.0};
  double startup_return_first_sample_timeout_s_{15.0};
  int startup_return_min_samples_{15};
  double startup_return_cluster_gap_m_{0.020};
  double startup_return_min_cluster_fraction_{0.50};
  double startup_return_min_half_width_m_{0.012};
  double startup_return_max_half_width_m_{0.030};
  double startup_return_margin_m_{0.005};
  double startup_return_default_max_range_m_{0.30};
  std::vector<double> startup_return_max_ranges_m_;
  std::string startup_return_authorization_topic_{"/control/planning_engaged"};
  std::chrono::steady_clock::time_point startup_return_started_at_;
  bool startup_return_learning_finalized_{false};
  bool startup_return_authorization_received_{false};
  bool startup_return_motion_authorized_{false};
  std::vector<std::vector<double>> startup_return_samples_;
  std::vector<double> startup_return_first_sample_elapsed_s_;
  std::vector<bool> startup_return_sensor_finalized_;
  std::size_t startup_return_learned_count_{0U};
  double cost_far_distance_m_{2.0};
  double obstacle_radius_m_{0.30};
  double ego_clear_radius_m_{0.50};
  double max_message_age_s_{0.35};
  double publish_rate_hz_{10.0};
  bool publish_radar_status_{false};
  double obstacle_evidence_warn_interval_s_{1.0};
  bool obstacle_evidence_warned_{false};
  std::chrono::steady_clock::time_point last_obstacle_evidence_warn_at_;
  bool route_lanelet_filter_enable_{true};
  std::string route_lanelet_mask_topic_{"/map/cost_grid/route_lanelet_mask"};
  double route_lanelet_margin_m_{0.35};
  int route_lanelet_allowed_max_cost_{50};
  double route_lanelet_mask_max_age_s_{2.5};
  bool route_lanelet_filter_fail_open_when_robot_outside_{true};
  bool route_lanelet_mask_has_allowed_cells_{false};
  std::vector<std::string> input_topics_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<avg_msgs::msg::AvgOccupancyGrid>::SharedPtr pub_grid_;
  rclcpp::Publisher<avg_msgs::msg::AvgSensingRadar>::SharedPtr avg_radar_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgString>::SharedPtr obstacle_evidence_pub_;
  rclcpp::Subscription<avg_msgs::msg::AvgOccupancyGrid>::SharedPtr route_lanelet_mask_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr
    global_dummy_active_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr
    startup_return_authorization_sub_;
  std::vector<rclcpp::Subscription<avg_msgs::msg::AvgRange>::SharedPtr> subs_;
  std::vector<rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr>
    channel_dummy_active_subs_;
  std::vector<RangeSample> samples_;
  DummyState global_dummy_state_;
  std::vector<DummyState> channel_dummy_states_;
  avg_msgs::msg::AvgOccupancyGrid::ConstSharedPtr route_lanelet_mask_;
  rclcpp::Time route_lanelet_mask_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    startup_only_param_callback_handle_;
};

}  // namespace camrod::sensing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod::sensing::RadarCostGridNode>());
  rclcpp::shutdown();
  return 0;
}
