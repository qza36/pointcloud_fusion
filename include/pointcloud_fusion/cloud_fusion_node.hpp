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
    void setTransformMatrix(Eigen::Matrix4f& transform, float roll, float pitch, float yaw, float tx, float ty, float tz);

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud1_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud2_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_cloud_pub_;


    std::string lidar_topic_{"livox/lidar_192_168_1_121"};
    std::string rear_lidar_topic_{"livox/lidar_192_168_1_178"};

    std::string target_frame_{"base_link"};

    // 变换参数（欧拉角 + 平移
    // 注意两个topic对应的变换矩阵的不同
    float roll1_ = 0.0f, pitch1_ = 0.0f, yaw1_ = 0.0f; // cloud1 的欧拉角（弧度）
    float tx1_ = 0.08f, ty1_ = 0.0f, tz1_ = 0.0f;       // cloud1 的平移（米）

    float roll2_ = 0.0f, pitch2_ = 0.0f, yaw2_ = 0.0f; // cloud2 的欧拉角（弧度）
    float tx2_ = 0.0f, ty2_ = 0.0f, tz2_ = 1.05f;       // cloud2 的平移（米）



    //同步策略
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> SyncPolicy;

    // 同步器
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

};