//
// Created by ZhiangQi on 2025/8/20.
//
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/transforms.hpp>
#include "pointcloud_fusion/cloud_fusion_node.hpp"


CloudFusion::CloudFusion(const rclcpp::NodeOptions& options):Node("CloudFusion",options)
{

    this->initializeParameters();

    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    fused_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points/fused", 10);

    // 订阅两个 Livox 点云话题
    cloud1_sub_.subscribe(this, lidar_topic_);
    cloud2_sub_.subscribe(this, rear_lidar_topic_);



    // 使用 ApproximateTime 同步器
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), cloud1_sub_, cloud2_sub_);
    sync_->registerCallback(
      std::bind(&CloudFusion::syncCallback, this, std::placeholders::_1, std::placeholders::_2));
}


void CloudFusion::initializeParameters()
{
    RCLCPP_INFO(this->get_logger(), "Initializing parameters");
    target_frame_ = this->declare_parameter<std::string>("target_frame", target_frame_);

    rear_lidar_frame_ = declare_parameter<std::string>("rear_lidar_frame", rear_lidar_frame_);
    lidar_frame_ = declare_parameter<std::string>("lidar_frame",lidar_frame_);

    lidar_topic_ = declare_parameter<std::string>("idar_topic",lidar_topic_);
    rear_lidar_topic_ = declare_parameter<std::string>("rear_lidar_topic",rear_lidar_topic_);
}
void CloudFusion::syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2_msg)
{
    RCLCPP_INFO(this->get_logger(),"start callback");
    // 将 ROS2 PointCloud2消息转换为 PCL 点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud1_msg, *cloud1);
    pcl::fromROSMsg(*cloud2_msg, *cloud2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    try {
        pcl_ros::transformPointCloud(target_frame_, *cloud1, *cloud1_transformed, *tf2_buffer_);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Cloud1 transform error: %s", ex.what());
        return;
    }

    // 转换第二个点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    try {
        pcl_ros::transformPointCloud(target_frame_, *cloud2, *cloud2_transformed, *tf2_buffer_);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Cloud2 transform error: %s", ex.what());
        return;
    }
    // === 拼接点云 ===
    *cloud1_transformed += *cloud2_transformed;


    sensor_msgs::msg::PointCloud2 fused_cloud_msg;
    pcl::toROSMsg(*cloud1_transformed, fused_cloud_msg);

    // 设置消息头
    fused_cloud_msg.header.stamp = cloud1_msg->header.stamp; // 使用其中一个的时间戳
    fused_cloud_msg.header.frame_id = target_frame_;

    fused_cloud_pub_->publish(fused_cloud_msg);
}
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<CloudFusion>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}