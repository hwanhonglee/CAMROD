#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

class GroundFilterNode : public rclcpp::Node
{
public:
  GroundFilterNode() : Node("vanjee_ground_filter")
  {
    method_ = "ransac";

    input_topic_  = "/vanjee_points722";
    nonground_topic_ = "/points_nonground";

    z_min_ = -0.25;
    z_max_ =  0.25;

    voxel_leaf_ = 0.20;
    ransac_dist_thresh_ = 0.12;
    ransac_max_iter_ = 120;

    // ✅ 최신 1개만 처리 (latency 줄이기)
    auto qos = rclcpp::SensorDataQoS().keep_last(1);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&GroundFilterNode::cb, this, std::placeholders::_1)
    );

    pub_nonground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      nonground_topic_, qos
    );

    RCLCPP_INFO(this->get_logger(), "Ground filter (nonground only) started");
  }

private:
  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr nonground(new pcl::PointCloud<pcl::PointXYZI>());

    if (method_ == "z")
    {
      for (const auto &p : cloud->points)
      {
        if (!(p.z >= z_min_ && p.z <= z_max_))
          nonground->points.push_back(p);
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
        *nonground = *cloud;
      }
      else
      {
        const double a = coeff->values[0];
        const double b = coeff->values[1];
        const double c = coeff->values[2];
        const double d = coeff->values[3];
        const double denom = std::sqrt(a*a + b*b + c*c);

        for (const auto &p : cloud->points)
        {
          double dist = std::fabs(a*p.x + b*p.y + c*p.z + d) / (denom + 1e-9);
          if (dist > ransac_dist_thresh_)
            nonground->points.push_back(p);
        }
      }
    }

    sensor_msgs::msg::PointCloud2 out_ng;
    pcl::toROSMsg(*nonground, out_ng);

    // ✅ RViz 지연 방지용: 현재 시간으로 stamp 갱신
    out_ng.header = msg->header;
    out_ng.header.stamp = this->get_clock()->now();

    pub_nonground_->publish(out_ng);
  }

  std::string method_;
  std::string input_topic_, nonground_topic_;

  double z_min_, z_max_;
  double voxel_leaf_;
  double ransac_dist_thresh_;
  int ransac_max_iter_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_nonground_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFilterNode>());
  rclcpp::shutdown();
  return 0;
}
