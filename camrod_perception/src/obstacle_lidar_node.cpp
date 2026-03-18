#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>
#include <limits>
#include <cmath>

struct AABB
{
  float min_x, min_y, min_z;
  float max_x, max_y, max_z;
  int count = 0;
};

class EuclideanBBoxNode : public rclcpp::Node
{
public:
  EuclideanBBoxNode() : Node("obstacle_lidar_node")
  {
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/sensing/lidar/points");
    bbox_topic_   = this->declare_parameter<std::string>("bbox_topic", "/perception/lidar/bboxes");

    cluster_tolerance_ = this->declare_parameter<double>("cluster_tolerance", 0.4);
    min_cluster_size_  = this->declare_parameter<int>("min_cluster_size", 10);
    max_cluster_size_  = this->declare_parameter<int>("max_cluster_size", 5000);

    use_box_ = this->declare_parameter<bool>("use_box", true);
    x_min_   = this->declare_parameter<double>("x_min", 0.0);
    x_max_   = this->declare_parameter<double>("x_max", 5.0);
    y_min_   = this->declare_parameter<double>("y_min", -3.0);
    y_max_   = this->declare_parameter<double>("y_max",  3.0);
    z_min_   = this->declare_parameter<double>("z_min", -3.0);
    z_max_   = this->declare_parameter<double>("z_max",  5.0);

    marker_lifetime_sec_ = this->declare_parameter<double>("marker_lifetime_sec", 0.15);
    draw_text_ = this->declare_parameter<bool>("draw_text", true);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&EuclideanBBoxNode::cb, this, std::placeholders::_1));

    pub_bbox_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(bbox_topic_, 10);

    RCLCPP_INFO(this->get_logger(),
      "Euclidean+BBox started. input=%s",
      input_topic_.c_str());
  }

private:
  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) return;

    // ROI 필터
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &p : cloud->points)
    {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

      if (use_box_)
      {
        if (p.x < x_min_ || p.x > x_max_) continue;
        if (p.y < y_min_ || p.y > y_max_) continue;
        if (p.z < z_min_ || p.z > z_max_) continue;
      }
      filtered->points.push_back(p);
    }

    if (filtered->empty()) return;

    // KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(filtered);

    // Euclidean Clustering
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered);
    ec.extract(cluster_indices);

    if (cluster_indices.empty())
    {
      publishDeleteAll(msg->header.frame_id, msg->header.stamp);
      return;
    }

    std::vector<AABB> boxes(cluster_indices.size());
    std::vector<float> min_dists(cluster_indices.size(), std::numeric_limits<float>::max());

    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
      auto &b = boxes[i];
      b.min_x = b.min_y = b.min_z = std::numeric_limits<float>::infinity();
      b.max_x = b.max_y = b.max_z = -std::numeric_limits<float>::infinity();
      b.count = 0;

      for (auto idx : cluster_indices[i].indices)
      {
        const auto &p = filtered->points[idx];

        b.min_x = std::min(b.min_x, p.x);
        b.min_y = std::min(b.min_y, p.y);
        b.min_z = std::min(b.min_z, p.z);

        b.max_x = std::max(b.max_x, p.x);
        b.max_y = std::max(b.max_y, p.y);
        b.max_z = std::max(b.max_z, p.z);

        b.count++;

        // 🔥 최소 거리 계산 (핵심)
        float d = std::sqrt(p.x*p.x + p.y*p.y);
        if (d < min_dists[i]) min_dists[i] = d;
      }
    }

    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker del;
    del.header = msg->header;
    del.ns = "euclidean_bbox";
    del.id = 0;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    int marker_id = 1;
    const rclcpp::Duration life =
      rclcpp::Duration::from_seconds(marker_lifetime_sec_);

    for (size_t c = 0; c < boxes.size(); ++c)
    {
      const auto &b = boxes[c];

      float sx = b.max_x - b.min_x;
      float sy = b.max_y - b.min_y;
      float sz = b.max_z - b.min_z;

      float cx = (b.min_x + b.max_x) * 0.5f;
      float cy = (b.min_y + b.max_y) * 0.5f;
      float cz = (b.min_z + b.max_z) * 0.5f;

      visualization_msgs::msg::Marker cube;
      cube.header = msg->header;
      cube.ns = "euclidean_bbox";
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
        text.ns = "euclidean_text";
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

        float dist = min_dists[c];  // 🔥 핵심 변경
        text.text =
          "id=" + std::to_string(c) +
          " n=" + std::to_string(b.count) +
          " d=" + std::to_string(dist);

        arr.markers.push_back(text);
      }
    }

    pub_bbox_->publish(arr);
  }

  void publishDeleteAll(const std::string &frame_id,
                        const rclcpp::Time &stamp)
  {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = frame_id;
    del.header.stamp = stamp;
    del.ns = "euclidean_bbox";
    del.id = 0;
    del.action = visualization_msgs::msg::Marker::DELETEALL;

    arr.markers.push_back(del);
    pub_bbox_->publish(arr);
  }

  std::string input_topic_, bbox_topic_;

  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;

  bool use_box_;
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;

  double marker_lifetime_sec_;
  bool draw_text_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_bbox_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EuclideanBBoxNode>());
  rclcpp::shutdown();
  return 0;
}