
#include <rtabmap_util/lidar_deskewing.hpp>

#include <laser_geometry/laser_geometry.hpp>

#include <rtabmap_conversions/MsgConversion.h>

namespace rtabmap_util
{

LidarDeskewing::LidarDeskewing(const rclcpp::NodeOptions & options) :
	Node("lidar_deskewing", options),
	waitForTransformDuration_(0.01),
	slerp_(false),
	clockwiseScan_(false)
{
	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, this);

	int queueSize = 5;
	int qos = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
	queueSize = this->declare_parameter("queue_size", queueSize);
	qos = this->declare_parameter("qos", qos);
	fixedFrameId_ = this->declare_parameter("fixed_frame_id", fixedFrameId_);
	waitForTransformDuration_ = this->declare_parameter("wait_for_transform", waitForTransformDuration_);
	slerp_ = this->declare_parameter("slerp", slerp_);
	const bool is2d = this->declare_parameter("is2d", true);
	clockwiseScan_ = this->declare_parameter("clockwise_scan", false);

	RCLCPP_INFO(this->get_logger(), "  fixed_frame_id:  %s", fixedFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "  wait_for_transform:  %fs", waitForTransformDuration_);
	RCLCPP_INFO(this->get_logger(), "  slerp:  %s", slerp_?"true":"false");
	RCLCPP_INFO(this->get_logger(), "  2d lidar:  %s", is2d?"true":"false");

	if(fixedFrameId_.empty())
	{
		RCLCPP_FATAL(this->get_logger(), "fixed_frame_id parameter cannot be empty!");
	}

	if(is2d)
	{
		subScan_ = create_subscription<sensor_msgs::msg::LaserScan>("input_scan", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos), std::bind(&LidarDeskewing::callbackScan, this, std::placeholders::_1));
		pubScan_ = create_publisher<sensor_msgs::msg::PointCloud2>(std::string(subScan_->get_topic_name()) + "/deskewed", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
	}
	else
	{
		subCloud_ = create_subscription<sensor_msgs::msg::PointCloud2>("input_cloud", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos), std::bind(&LidarDeskewing::callbackCloud, this, std::placeholders::_1));
		pubCloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(std::string(subCloud_->get_topic_name()) + "/deskewed", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
	}
}

LidarDeskewing::~LidarDeskewing()
{
}

void LidarDeskewing::callbackScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
	if(scanSyncDiagnostic_.get() == 0) {
		scanSyncDiagnostic_.reset(new rtabmap_sync::SyncDiagnostic(this, 0.5));
		scanSyncDiagnostic_->init(subScan_->get_topic_name(),
			uFormat("%s: Did not receive data since 5 seconds! Make sure the input topic \"%s\" is "
						"published (\"$ rostopic hz my_topic\") and the timestamps in their "
						"header are set.",
						this->get_name(),
						subScan_->get_topic_name()));
	}
	scanSyncDiagnostic_->tickInput(msg->header.stamp);

	sensor_msgs::msg::PointCloud2::UniquePtr tmpCloud(new sensor_msgs::msg::PointCloud2);
	sensor_msgs::msg::PointCloud2::UniquePtr deskewedCloud(new sensor_msgs::msg::PointCloud2);
	rtabmap_conversions::laserScanToPointCloud(*msg, *tmpCloud, clockwiseScan_);
	// deskew process transforms points to the spatiotemporal coordinate of the last point of the scan
	// frame ID of deskewedCloud remains the same as original msg
	if(!rtabmap_conversions::deskew(*tmpCloud, *deskewedCloud, fixedFrameId_, *tfBuffer_, waitForTransformDuration_, slerp_))
	{
		deskewedCloud = std::move(tmpCloud);
	}
	pubScan_->publish(std::move(deskewedCloud));

	scanSyncDiagnostic_->tickOutput(msg->header.stamp);
}

void LidarDeskewing::callbackCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
	if(cloudSyncDiagnostic_.get() == 0) {
		cloudSyncDiagnostic_.reset(new rtabmap_sync::SyncDiagnostic(this, 0.5));
		cloudSyncDiagnostic_->init(subCloud_->get_topic_name(),
			uFormat("%s: Did not receive data since 5 seconds! Make sure the input topic \"%s\" is "
						"published (\"$ rostopic hz my_topic\") and the timestamps in their "
						"header are set.",
						this->get_name(),
						subCloud_->get_topic_name()));
	}
	cloudSyncDiagnostic_->tickInput(msg->header.stamp);

	sensor_msgs::msg::PointCloud2 msgDeskewed;
	if(rtabmap_conversions::deskew(*msg, msgDeskewed, fixedFrameId_, *tfBuffer_, waitForTransformDuration_, slerp_))
	{
		pubCloud_->publish(msgDeskewed);
	}
	else
	{
		// Just republish the msg to not breakdown downstream
		// A warning should be already shown (see deskew() source code)
		RCLCPP_WARN(this->get_logger(), "Deskewing failed. Relaying original cloud...");
		pubCloud_->publish(*msg);
	}
	cloudSyncDiagnostic_->tickOutput(msg->header.stamp);
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_util::LidarDeskewing)
