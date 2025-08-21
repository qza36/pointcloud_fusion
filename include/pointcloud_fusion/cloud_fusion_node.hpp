#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>



class CloudFusion : public rclcpp::Node
{
public:
    CloudFusion() = delete;
    CloudFusion(const rclcpp::NodeOptions& options);
private:
    void initializeParameters();
    void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2_msg);

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud1_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud2_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_cloud_pub_;


    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    std::string lidar_frame_{"livox_frame_bottom"};
    std::string rear_lidar_frame_{"livox_frame_top"};

    std::string lidar_topic_{"/BottomLidar/livox/lidar/pointcloud"};
    std::string rear_lidar_topic_{"/TopLidar/livox/lidar/pointcloud"};

    std::string target_frame_{"base_link"};




    //同步策略
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> SyncPolicy;

    // 同步器
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

};