#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <string>

struct Pt { float x, y, z; };

static inline float sqr(float v)
{
  return v * v;
}

struct AABB
{
  float min_x, min_y, min_z;
  float max_x, max_y, max_z;
  int count = 0;
};

class DBSCANBBoxNode : public rclcpp::Node
{
public:
  DBSCANBBoxNode() : Node("obstacle_lidar_node")
  {
    // ---- 기본값 ----
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/sensing/lidar/points");
    bbox_topic_   = this->declare_parameter<std::string>("bbox_topic", "/perception/lidar/bboxes");

    eps_          = this->declare_parameter<double>("eps", 0.25);
    min_pts_      = this->declare_parameter<int>("min_pts", 12);
    max_points_   = this->declare_parameter<int>("max_points", 8000);

    use_box_ = this->declare_parameter<bool>("use_box", true);
    x_min_   = this->declare_parameter<double>("x_min", 0.0);
    x_max_   = this->declare_parameter<double>("x_max", 5.0);
    y_min_   = this->declare_parameter<double>("y_min", -3.0);
    y_max_   = this->declare_parameter<double>("y_max",  3.0);
    z_min_   = this->declare_parameter<double>("z_min", -3.0);
    z_max_   = this->declare_parameter<double>("z_max",  5.0);

    min_cluster_points_ = this->declare_parameter<int>("min_cluster_points", 25);
    min_size_x_ = this->declare_parameter<double>("min_size_x", 0.10);
    min_size_y_ = this->declare_parameter<double>("min_size_y", 0.10);
    min_size_z_ = this->declare_parameter<double>("min_size_z", 0.10);

    marker_lifetime_sec_ = this->declare_parameter<double>("marker_lifetime_sec", 0.15);
    draw_text_ = this->declare_parameter<bool>("draw_text", true);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DBSCANBBoxNode::cb, this, std::placeholders::_1));

    pub_bbox_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(bbox_topic_, 10);

    RCLCPP_INFO(this->get_logger(),
      "DBSCAN+BBox started. input=%s bbox=%s eps=%.3f min_pts=%d",
      input_topic_.c_str(), bbox_topic_.c_str(), eps_, min_pts_);
  }

private:
  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    bool has_x = false, has_y = false, has_z = false;

    for (const auto &f : msg->fields)
    {
      if (f.name == "x") has_x = true;
      if (f.name == "y") has_y = true;
      if (f.name == "z") has_z = true;
    }

    if (!(has_x && has_y && has_z)) return;

    std::vector<Pt> pts;
    pts.reserve(std::min<int>(max_points_, (int)(msg->width * msg->height)));

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    int taken = 0;

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
    {
      if (taken >= max_points_) break;

      float x = *it_x;
      float y = *it_y;
      float z = *it_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

      if (use_box_)
      {
        if (x < x_min_ || x > x_max_) continue;
        if (y < y_min_ || y > y_max_) continue;
        if (z < z_min_ || z > z_max_) continue;
      }

      pts.push_back({x, y, z});
      taken++;
    }

    if ((int)pts.size() < min_pts_)
    {
      publishDeleteAll(msg->header.frame_id, msg->header.stamp);
      return;
    }

    const float eps2 = (float)(eps_ * eps_);
    std::vector<int> labels(pts.size(), -1);

    auto regionQuery = [&](size_t idx, std::vector<size_t> &neighbors)
    {
      neighbors.clear();
      const auto &p = pts[idx];

      for (size_t j = 0; j < pts.size(); ++j)
      {
        const auto &q = pts[j];
        float d2 = sqr(p.x - q.x) + sqr(p.y - q.y) + sqr(p.z - q.z);
        if (d2 <= eps2) neighbors.push_back(j);
      }
    };

    int cluster_id = 0;
    std::vector<size_t> neighbors, neighbors2;
    neighbors.reserve(128);
    neighbors2.reserve(128);

    for (size_t i = 0; i < pts.size(); ++i)
    {
      if (labels[i] != -1) continue;

      regionQuery(i, neighbors);

      if ((int)neighbors.size() < min_pts_)
      {
        labels[i] = -2;
        continue;
      }

      int cid = cluster_id++;
      labels[i] = cid;

      std::queue<size_t> q;
      for (auto n : neighbors) q.push(n);

      while (!q.empty())
      {
        size_t cur = q.front();
        q.pop();

        if (labels[cur] == -2) labels[cur] = cid;
        if (labels[cur] != -1) continue;

        labels[cur] = cid;

        regionQuery(cur, neighbors2);
        if ((int)neighbors2.size() >= min_pts_)
          for (auto n2 : neighbors2) q.push(n2);
      }
    }

    if (cluster_id <= 0)
    {
      publishDeleteAll(msg->header.frame_id, msg->header.stamp);
      return;
    }

    std::vector<AABB> boxes(cluster_id);

    for (int c = 0; c < cluster_id; ++c)
    {
      boxes[c].min_x = boxes[c].min_y = boxes[c].min_z =
        std::numeric_limits<float>::infinity();
      boxes[c].max_x = boxes[c].max_y = boxes[c].max_z =
        -std::numeric_limits<float>::infinity();
      boxes[c].count = 0;
    }

    for (size_t i = 0; i < pts.size(); ++i)
    {
      int lab = labels[i];
      if (lab < 0) continue;

      auto &b = boxes[lab];

      b.min_x = std::min(b.min_x, pts[i].x);
      b.min_y = std::min(b.min_y, pts[i].y);
      b.min_z = std::min(b.min_z, pts[i].z);

      b.max_x = std::max(b.max_x, pts[i].x);
      b.max_y = std::max(b.max_y, pts[i].y);
      b.max_z = std::max(b.max_z, pts[i].z);

      b.count++;
    }

    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker del;
    del.header = msg->header;
    del.ns = "dbscan_bbox";
    del.id = 0;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    int marker_id = 1;
    const rclcpp::Duration life =
      rclcpp::Duration::from_seconds(marker_lifetime_sec_);

    for (int c = 0; c < cluster_id; ++c)
    {
      const auto &b = boxes[c];
      if (b.count < min_cluster_points_) continue;

      float sx = b.max_x - b.min_x;
      float sy = b.max_y - b.min_y;
      float sz = b.max_z - b.min_z;

      if (sx < min_size_x_ || sy < min_size_y_ || sz < min_size_z_) continue;

      float cx = (b.min_x + b.max_x) * 0.5f;
      float cy = (b.min_y + b.max_y) * 0.5f;
      float cz = (b.min_z + b.max_z) * 0.5f;

      visualization_msgs::msg::Marker cube;
      cube.header = msg->header;
      cube.ns = "dbscan_bbox";
      cube.id = marker_id++;
      cube.type = visualization_msgs::msg::Marker::CUBE;
      cube.action = visualization_msgs::msg::Marker::ADD;
      cube.lifetime = life;

      cube.pose.position.x = cx;
      cube.pose.position.y = cy;
      cube.pose.position.z = cz;
      cube.pose.orientation.w = 1.0;

      cube.scale.x = std::max(sx, 0.01f);
      cube.scale.y = std::max(sy, 0.01f);
      cube.scale.z = std::max(sz, 0.01f);

      cube.color.a = 0.45f;
      cube.color.r = (float)((c * 97) % 255) / 255.0f;
      cube.color.g = (float)((c * 57) % 255) / 255.0f;
      cube.color.b = (float)((c * 17) % 255) / 255.0f;

      arr.markers.push_back(cube);

      if (draw_text_)
      {
        visualization_msgs::msg::Marker text;
        text.header = msg->header;
        text.ns = "dbscan_text";
        text.id = marker_id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.lifetime = life;

        text.pose.position.x = cx;
        text.pose.position.y = cy;
        text.pose.position.z = cz + (cube.scale.z * 0.5f) + 0.1f;
        text.pose.orientation.w = 1.0;

        text.scale.z = 0.20;
        text.color.a = 1.0;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;

        float dist = std::sqrt(cx * cx + cy * cy);
        text.text =
          "id=" + std::to_string(c) +
          " n=" + std::to_string(b.count) +
          " d=" + std::to_string(dist);

        arr.markers.push_back(text);
      }
    }

    pub_bbox_->publish(arr);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "DBSCAN: pts=%zu clusters=%d (bbox published)",
      pts.size(), cluster_id);
  }

  void publishDeleteAll(const std::string &frame_id,
                        const rclcpp::Time &stamp)
  {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = frame_id;
    del.header.stamp = stamp;
    del.ns = "dbscan_bbox";
    del.id = 0;
    del.action = visualization_msgs::msg::Marker::DELETEALL;

    arr.markers.push_back(del);
    pub_bbox_->publish(arr);
  }

  std::string input_topic_, bbox_topic_;
  double eps_;
  int min_pts_;
  int max_points_;

  bool use_box_;
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;

  int min_cluster_points_;
  double min_size_x_, min_size_y_, min_size_z_;

  double marker_lifetime_sec_;
  bool draw_text_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_bbox_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DBSCANBBoxNode>());
  rclcpp::shutdown();
  return 0;
}