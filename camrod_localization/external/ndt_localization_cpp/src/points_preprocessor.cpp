// LiDAR points -> NDT input (C++ port of ndt_localization/points_preprocessor.py, the features camrod uses):
//   drop non-finite points and points outside min_range..max_range, voxel-grid centroids, and move the
//   header stamp to the LAST point (time_field, seconds relative to the stamp; vanjee stamps the first point,
//   which made the NDT pose ~0.1 s early). The cloud stays in the sensor frame (ndt_scan_matcher uses TF).
// Output is byte-identical to the Python node: voxels in x, y, z index order (= np.unique(axis=0)), centroid sums
// accumulated in point order in double, written as float32.
//   in  ~/input  sensor_msgs/PointCloud2 (any layout with float x y z)   out ~/output  x y z float32
// Parameters: min_range, max_range [m], voxel_size [m] (0 = no downsampling), time_field ('' = keep the stamp)
//   output_frame_id ('' = keep the input frame): NDT-only LiDAR frame. When set, the output cloud carries this frame
//   and the node publishes the static TF output_frame_parent -> output_frame_id (output_frame_xyz [m],
//   output_frame_rpy [rad]). Used to correct the LiDAR mount for NDT without touching the robot TF.
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace ndt_localization_cpp
{

class PointsPreprocessor : public rclcpp::Node
{
public:
  explicit PointsPreprocessor(const rclcpp::NodeOptions & options)
  : Node("points_preprocessor", options)
  {
    min_r_ = declare_parameter<double>("min_range", 1.0);
    max_r_ = declare_parameter<double>("max_range", 100.0);
    voxel_ = declare_parameter<double>("voxel_size", 1.0);
    time_field_ = declare_parameter<std::string>("time_field", "");
    // the Python node's ground split / deskew are not ported (camrod runs them off)
    if (declare_parameter<double>("ground_voxel_size", 0.0) > 0.0 || declare_parameter<bool>("deskew", false)) {
      RCLCPP_WARN(get_logger(), "ground_voxel_size / deskew are not supported in the C++ node: ignored");
    }
    output_frame_ = declare_parameter<std::string>("output_frame_id", "");
    const auto parent = declare_parameter<std::string>("output_frame_parent", "robot_center_link");
    const auto xyz = declare_parameter<std::vector<double>>("output_frame_xyz", {0.0, 0.0, 0.0});
    const auto rpy = declare_parameter<std::vector<double>>("output_frame_rpy", {0.0, 0.0, 0.0});
    if (!output_frame_.empty()) {
      if (xyz.size() != 3 || rpy.size() != 3) {
        throw std::invalid_argument("output_frame_xyz / output_frame_rpy need 3 values");
      }
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now();
      t.header.frame_id = parent;
      t.child_frame_id = output_frame_;
      t.transform.translation.x = xyz[0];
      t.transform.translation.y = xyz[1];
      t.transform.translation.z = xyz[2];
      const double cr = std::cos(rpy[0] / 2), sr = std::sin(rpy[0] / 2), cp = std::cos(rpy[1] / 2),
                   sp = std::sin(rpy[1] / 2), cy = std::cos(rpy[2] / 2), sy = std::sin(rpy[2] / 2);
      t.transform.rotation.w = cr * cp * cy + sr * sp * sy;
      t.transform.rotation.x = sr * cp * cy - cr * sp * sy;
      t.transform.rotation.y = cr * sp * cy + sr * cp * sy;
      t.transform.rotation.z = cr * cp * sy - sr * sp * cy;
      tf_static_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
      tf_static_->sendTransform(t);
      RCLCPP_INFO(get_logger(), "NDT LiDAR frame: %s -> %s xyz %.3f %.3f %.3f rpy %.3f %.3f %.3f (robot TF unchanged)",
                  parent.c_str(), output_frame_.c_str(), xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);
    }
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/output", rclcpp::SensorDataQoS());
    // reliable input: a Vanjee cloud is ~1.3 MB = many DDS fragments; over best effort one lost fragment drops it
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/input", rclcpp::QoS(5).reliable(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { on_points(*m); });
    timer_ = create_wall_timer(std::chrono::seconds(10), [this]() { report(); });
    RCLCPP_INFO(get_logger(), "range %.1f..%.1f m, voxel %.2f m, time field '%s'", min_r_, max_r_, voxel_,
                time_field_.c_str());
  }

private:
  struct Field
  {
    int offset{-1};
    uint8_t type{0};
  };

  static Field find(const sensor_msgs::msg::PointCloud2 & m, const std::string & name)
  {
    for (const auto & f : m.fields) {
      if (f.name == name && (f.datatype == sensor_msgs::msg::PointField::FLOAT32 ||
                             f.datatype == sensor_msgs::msg::PointField::FLOAT64)) {
        return {static_cast<int>(f.offset), f.datatype};
      }
    }
    return {};
  }

  static double read(const uint8_t * p, const Field & f)
  {
    if (f.type == sensor_msgs::msg::PointField::FLOAT64) {
      double v;
      std::memcpy(&v, p + f.offset, sizeof v);
      return v;
    }
    float v;
    std::memcpy(&v, p + f.offset, sizeof v);
    return v;
  }

  void on_points(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const Field fx = find(msg, "x"), fy = find(msg, "y"), fz = find(msg, "z");
    if (fx.offset < 0 || fy.offset < 0 || fz.offset < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cloud without float x/y/z: dropped");
      return;
    }
    const size_t n = static_cast<size_t>(msg.width) * msg.height;
    const uint8_t * base = msg.data.data();
    // ---- range filter (float64, as the Python node) ----
    std::vector<double> xyz;
    xyz.reserve(n * 3);
    for (size_t i = 0; i < n; ++i) {
      const uint8_t * p = base + i * msg.point_step;
      const double x = read(p, fx), y = read(p, fy), z = read(p, fz);
      const double r = std::sqrt(x * x + y * y + z * z);
      if (std::isfinite(r) && r > min_r_ && r < max_r_) {
        xyz.push_back(x);
        xyz.push_back(y);
        xyz.push_back(z);
      }
    }
    const size_t nv = xyz.size() / 3;
    ++n_msgs_;
    n_in_ += n;
    n_valid_ += nv;
    // ---- voxel centroids ----
    std::vector<float> out;
    if (voxel_ > 0.0 && nv > 0) {
      std::vector<std::array<int64_t, 3>> key(nv);
      for (size_t i = 0; i < nv; ++i) {
        for (int k = 0; k < 3; ++k) {
          key[i][k] = static_cast<int64_t>(std::floor(xyz[3 * i + k] / voxel_));
        }
      }
      std::vector<uint32_t> order(nv);
      std::iota(order.begin(), order.end(), 0u);
      std::stable_sort(order.begin(), order.end(), [&key](uint32_t a, uint32_t b) { return key[a] < key[b]; });
      out.reserve(nv);
      size_t i = 0;
      while (i < nv) {
        size_t j = i;
        double s[3] = {0.0, 0.0, 0.0};
        while (j < nv && key[order[j]] == key[order[i]]) {          // stable: point order inside a voxel
          for (int k = 0; k < 3; ++k) {
            s[k] += xyz[3 * order[j] + k];
          }
          ++j;
        }
        const double c = static_cast<double>(j - i);
        for (int k = 0; k < 3; ++k) {
          out.push_back(static_cast<float>(s[k] / c));
        }
        i = j;
      }
    } else {
      out.assign(xyz.begin(), xyz.end());
    }
    const size_t no = out.size() / 3;
    n_out_ += no;
    // ---- output ----
    auto o = std::make_unique<sensor_msgs::msg::PointCloud2>();
    o->header = msg.header;
    if (!output_frame_.empty()) {
      o->header.frame_id = output_frame_;
    }
    if (!time_field_.empty()) {
      const double t_last = last_point_time(msg, time_field_);
      if (t_last > 0.0) {
        const int64_t ns = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec +
                           static_cast<int64_t>(std::nearbyint(t_last * 1e9));
        o->header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
        o->header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
      }
    }
    o->height = 1;
    o->width = static_cast<uint32_t>(no);
    o->fields.resize(3);
    const char * names[3] = {"x", "y", "z"};
    for (int k = 0; k < 3; ++k) {
      o->fields[k].name = names[k];
      o->fields[k].offset = 4 * k;
      o->fields[k].datatype = sensor_msgs::msg::PointField::FLOAT32;
      o->fields[k].count = 1;
    }
    o->is_bigendian = false;
    o->point_step = 12;
    o->row_step = static_cast<uint32_t>(12 * no);
    o->is_dense = true;
    o->data.resize(12 * no);
    std::memcpy(o->data.data(), out.data(), 12 * no);
    pub_->publish(std::move(o));
  }

  // largest finite per-point time [s] (relative to the stamp); NaN points of vanjee carry t = 0 and a real sweep
  // is at most a few 100 ms, so anything outside (0, 1) s means "no usable time"
  static double last_point_time(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
  {
    const Field f = find(msg, name);
    if (f.offset < 0) {
      return 0.0;
    }
    const size_t n = static_cast<size_t>(msg.width) * msg.height;
    double mx = -std::numeric_limits<double>::infinity();
    bool any = false;
    for (size_t i = 0; i < n; ++i) {
      const double t = read(msg.data.data() + i * msg.point_step, f);
      if (std::isfinite(t)) {
        any = true;
        mx = std::max(mx, t);
      }
    }
    return (any && mx > 0.0 && mx < 1.0) ? mx : 0.0;
  }

  void report()
  {
    if (n_msgs_) {
      RCLCPP_INFO(get_logger(), "%zu scans, points per scan %zu -> %zu valid -> %zu", n_msgs_, n_in_ / n_msgs_,
                  n_valid_ / n_msgs_, n_out_ / n_msgs_);
      n_msgs_ = n_in_ = n_valid_ = n_out_ = 0;
    }
  }

  double min_r_, max_r_, voxel_;
  std::string time_field_, output_frame_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;
  size_t n_msgs_{0}, n_in_{0}, n_valid_{0}, n_out_{0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ndt_localization_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ndt_localization_cpp::PointsPreprocessor)
