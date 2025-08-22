//
// Created by ZhiangQi on 2025/8/20.
//
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/transforms.hpp>
#include "pointcloud_fusion/cloud_fusion_node.hpp"


CloudFusion::CloudFusion(const rclcpp::NodeOptions& options):Node("CloudFusion",options)
{

    this->initializeParameters();

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

    lidar_topic_ = declare_parameter<std::string>("lidar_topic",lidar_topic_);
    rear_lidar_topic_ = declare_parameter<std::string>("rear_lidar_topic",rear_lidar_topic_);

    tx1_ = this->declare_parameter<double>("cloud1.transform.tx", 0.08f);
    ty1_ = this->declare_parameter<double>("cloud1.transform.ty", 0.0);
    tz1_ = this->declare_parameter<double>("cloud1.transform.tz", 0.0);
    roll1_ = this->declare_parameter<double>("cloud1.transform.roll", 0.0);
    pitch1_ = this->declare_parameter<double>("cloud1.transform.pitch", 0.0);
    yaw1_ = this->declare_parameter<double>("cloud1.transform.yaw", 0.0);

    tx2_ = this->declare_parameter<double>("cloud2.transform.tx", 0.0);
    ty2_ = this->declare_parameter<double>("cloud2.transform.ty", 0.0);
    tz2_ = this->declare_parameter<double>("cloud2.transform.tz", 1.05f);
    roll2_ = this->declare_parameter<double>("cloud2.transform.roll", 0.0);
    pitch2_ = this->declare_parameter<double>("cloud2.transform.pitch", 0.0);
    yaw2_ = this->declare_parameter<double>("cloud2.transform.yaw", 0.0);

        RCLCPP_INFO(this->get_logger(), "Loaded transform for cloud1: [tx: %f, ty: %f, tz: %f, roll: %f, pitch: %f, yaw: %f]",
        tx1_, ty1_, tz1_, roll1_, pitch1_, yaw1_);
    RCLCPP_INFO(this->get_logger(), "Loaded transform for cloud2: [tx: %f, ty: %f, tz: %f, roll: %f, pitch: %f, yaw: %f]",
        tx2_, ty2_, tz2_, roll2_, pitch2_, yaw2_);
}
void CloudFusion::syncCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2_msg)
{
    RCLCPP_INFO(this->get_logger(),"start callback");
    // 将 ROS2 PointCloud2消息转换为 PCL 点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud1_msg, *cloud1);
    pcl::fromROSMsg(*cloud2_msg, *cloud2);


    // 定义变换矩阵
    Eigen::Matrix4f transform1 = Eigen::Matrix4f::Identity(); // cloud1 的变换矩阵
    Eigen::Matrix4f transform2 = Eigen::Matrix4f::Identity(); // cloud2 的变换矩阵


    // 设置 cloud1 的变换矩阵（欧拉角 + 平移）
    setTransformMatrix(transform1, roll1_, pitch1_, yaw1_, tx1_, ty1_, tz1_);

    // 设置 cloud2 的变换矩阵（欧拉角 + 平移）
    setTransformMatrix(transform2, roll2_, pitch2_, yaw2_, tx2_, ty2_, tz2_);


    // 对点云进行坐标变换
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud1, *cloud1_transformed, transform1);
    pcl::transformPointCloud(*cloud2, *cloud2_transformed, transform2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    *merged_cloud = *cloud1_transformed + *cloud2_transformed;


    sensor_msgs::msg::PointCloud2 merged_msg;
    pcl::toROSMsg(*merged_cloud, merged_msg);
    merged_msg.header.frame_id = target_frame_; // 设置坐标系
    merged_msg.header.stamp = cloud1_msg->header.stamp;


    fused_cloud_pub_->publish(merged_msg);
}
void CloudFusion::setTransformMatrix(Eigen::Matrix4f& transform, float roll,
    float pitch, float yaw,float tx, float ty, float tz)
{
    // 计算旋转矩阵
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3f rotationMatrix = q.matrix();

    // 设置变换矩阵的旋转部分
    transform.block<3, 3>(0, 0) = rotationMatrix;

    // 设置变换矩阵的平移部分
    transform(0, 3) = tx;
    transform(1, 3) = ty;
    transform(2, 3) = tz;

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