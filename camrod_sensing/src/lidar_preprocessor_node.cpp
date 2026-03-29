#include <cmath>
#include <memory>
#include <string>

#include <avg_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

class GroundFilterNode : public rclcpp::Node
{
public:
  GroundFilterNode() : Node("lidar_preprocessor")
  {
    this->declare_parameter<std::string>("input_topic", "vanjee/points_raw");
    this->declare_parameter<std::string>("filtered_topic", "points_filtered");
    this->declare_parameter<std::string>("lidar_status_topic", "status");

    this->declare_parameter<std::string>("method", "ransac");
    this->declare_parameter<double>("z_min", -0.25);
    this->declare_parameter<double>("z_max", 0.25);
    this->declare_parameter<double>("voxel_leaf", 0.10);
    this->declare_parameter<double>("ransac_dist_thresh", 0.12);
    this->declare_parameter<int>("ransac_max_iter", 120);

    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("filtered_topic", filtered_topic_);
    this->get_parameter("lidar_status_topic", lidar_status_topic_);

    this->get_parameter("method", method_);
    this->get_parameter("z_min", z_min_);
    this->get_parameter("z_max", z_max_);
    this->get_parameter("voxel_leaf", voxel_leaf_);
    this->get_parameter("ransac_dist_thresh", ransac_dist_thresh_);
    this->get_parameter("ransac_max_iter", ransac_max_iter_);

    auto qos = rclcpp::SensorDataQoS().keep_last(1);

    // HH_260326: Keep intra-stack lidar transport aligned on avg_msgs aliases.
    sub_ = this->create_subscription<avg_msgs::msg::PointCloud2>(
      input_topic_,
      qos,
      std::bind(&GroundFilterNode::cb, this, std::placeholders::_1)
    );

    pub_filtered_ = this->create_publisher<avg_msgs::msg::PointCloud2>(
      filtered_topic_,
      qos
    );

    RCLCPP_INFO(this->get_logger(), "Ground filter started");
    RCLCPP_INFO(this->get_logger(), "  input_topic    : %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  filtered_topic : %s", filtered_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  method         : %s", method_.c_str());
  }

private:
  void cb(const avg_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());

    if (method_ == "z")
    {
      for (const auto & p : cloud->points)
      {
        if (!(p.z >= z_min_ && p.z <= z_max_)) {
          filtered->points.push_back(p);
        }
      }
    }
    else
    {
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);

      pcl::PointCloud<pcl::PointXYZI>::Ptr ds(new pcl::PointCloud<pcl::PointXYZI>());
      vg.filter(*ds);

      pcl::SACSegmentation<pcl::PointXYZI> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(ransac_max_iter_);
      seg.setDistanceThreshold(ransac_dist_thresh_);
      seg.setInputCloud(ds);

      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
      seg.segment(*inliers, *coeff);

      if (inliers->indices.empty())
      {
        *filtered = *ds;
      }
      else
      {
        const double a = coeff->values[0];
        const double b = coeff->values[1];
        const double c = coeff->values[2];
        const double d = coeff->values[3];
        const double denom = std::sqrt(a * a + b * b + c * c);

        for (const auto & p : ds->points)
        {
          const double dist = std::fabs(a * p.x + b * p.y + c * p.z + d) / (denom + 1e-9);
          if (dist > ransac_dist_thresh_) {
            filtered->points.push_back(p);
          }
        }
      }
    }

    avg_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*filtered, out);
    out.header = msg->header;
    out.header.stamp = this->get_clock()->now();

    pub_filtered_->publish(out);
  }

  std::string input_topic_;
  std::string filtered_topic_;
  std::string lidar_status_topic_;

  std::string method_;
  double z_min_;
  double z_max_;
  double voxel_leaf_;
  double ransac_dist_thresh_;
  int ransac_max_iter_;

  rclcpp::Subscription<avg_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<avg_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFilterNode>());
  rclcpp::shutdown();
  return 0;
}
