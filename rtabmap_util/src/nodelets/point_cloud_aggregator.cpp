/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap_util/point_cloud_aggregator.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/util3d_filtering.h>

namespace rtabmap_util
{

PointCloudAggregator::PointCloudAggregator(const rclcpp::NodeOptions & options) :
	Node("point_cloud_aggregator", options),
	warningThread_(0),
	callbackCalled_(false),
	exactSync4_(0),
	approxSync4_(0),
	exactSync3_(0),
	approxSync3_(0),
	exactSync2_(0),
	approxSync2_(0),
	waitForTransform_(0.1),
	xyzOutput_(false),
	convertToLaserScan_(false),
	scanAngleMin_(-M_PI),
	scanAngleMax_(M_PI),
	scanAngleIncrement_(0.0174533),
	scanRangeMin_(0.0),
	scanRangeMax_(1000.0)
{
	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	//auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
	//	this->get_node_base_interface(),
	//	this->get_node_timers_interface());
	//tfBuffer_->setCreateTimerInterface(timer_interface);
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, this);

	int topicQueueSize = 1;
	int syncQueueSize = 5;
	int count = 2;
	bool approx=true;
	double approxSyncMaxInterval = 0.0;
	double interMessageLowerBound = 0.0;
	double agePenalty = 1.0;
	int qos=RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
	topicQueueSize = this->declare_parameter("topic_queue_size", topicQueueSize);
	int queueSize = this->declare_parameter("queue_size", -1);
	if(queueSize != -1)
	{
		syncQueueSize = queueSize;
		RCLCPP_WARN(this->get_logger(), "Parameter \"queue_size\" has been renamed "
				 "to \"sync_queue_size\" and will be removed "
				 "in future versions! The value (%d) is copied to "
				 "\"sync_queue_size\".", syncQueueSize);
	}
	syncQueueSize = this->declare_parameter("sync_queue_size", syncQueueSize);
	qos = this->declare_parameter("qos", qos);
	frameId_ = this->declare_parameter("frame_id", frameId_);
	fixedFrameId_ = this->declare_parameter("fixed_frame_id", fixedFrameId_);
	approx = this->declare_parameter("approx_sync", approx);
	approxSyncMaxInterval = this->declare_parameter("approx_sync_max_interval", approxSyncMaxInterval);
	interMessageLowerBound = this->declare_parameter("inter_message_lower_bound", interMessageLowerBound);
	agePenalty = this->declare_parameter("age_penalty", agePenalty);
	count = this->declare_parameter("count", count);
	waitForTransform_ = this->declare_parameter("wait_for_transform", waitForTransform_);
	xyzOutput_ = this->declare_parameter("xyz_output", xyzOutput_);

	convertToLaserScan_ = this->declare_parameter("2d_output", convertToLaserScan_);
	scanAngleMin_ = this->declare_parameter("scan_angle_min", scanAngleMin_);
	scanAngleMax_ = this->declare_parameter("scan_angle_max", scanAngleMax_);
	if (scanAngleMin_ >= scanAngleMax_) {
		RCLCPP_WARN(this->get_logger(), "scan_angle_min(%f) must be smaller than scan_angle_max(%f). Swapping the values.", scanAngleMin_, scanAngleMax_);
		std::swap(scanAngleMin_, scanAngleMax_);
	}
	scanAngleIncrement_ = this->declare_parameter("scan_angle_increment", scanAngleIncrement_);
	if (scanAngleIncrement_ < 0.0001) {
		RCLCPP_WARN(this->get_logger(), "scan_angle_increment(%f) must be larger than 0. Setting it to 0.0174533(1 degrees).", scanAngleIncrement_);
		scanAngleIncrement_ = 0.0174533;
	}
	scanRangeMin_ = this->declare_parameter("scan_range_min", scanRangeMin_);
	scanRangeMax_ = this->declare_parameter("scan_range_max", scanRangeMax_);
	if (scanRangeMin_ <= 0 or scanRangeMax_ <= 0) {
		RCLCPP_WARN(this->get_logger(), "scan_range_min(%f) and scan_range_max(%f) must be larger than 0.0. Swapping signs.", scanRangeMin_, scanRangeMax_);
		scanRangeMin_ = std::abs(scanRangeMin_);
		scanRangeMax_ = std::abs(scanRangeMax_);
	}
	if (scanRangeMin_ >= scanRangeMax_) {
		RCLCPP_WARN(this->get_logger(), "scan_range_min(%f) must be smaller than scan_range_max(%f). Swapping the values.", scanRangeMin_, scanRangeMax_);
		std::swap(scanRangeMin_, scanRangeMax_);
	}
	numPoints_ = std::ceil((scanAngleMax_ - scanAngleMin_) / scanAngleIncrement_);
	if (numPoints_ < 50) {
		RCLCPP_WARN(this->get_logger(), "Number of points in combined LaserScan message is too small(%d). Recommended is at least 200. Check your parameters.", numPoints_);
	}

	if (convertToLaserScan_) {
		scanPub_ = create_publisher<sensor_msgs::msg::LaserScan>("combined_scan", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
	} else {
		cloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>("combined_cloud", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
	}

	cloudSub_1_.subscribe(this, "cloud1", RCLCPP_QOS(topicQueueSize, qos));
	cloudSub_2_.subscribe(this, "cloud2", RCLCPP_QOS(topicQueueSize, qos));

	if(count == 4)
	{
		cloudSub_3_.subscribe(this, "cloud3", RCLCPP_QOS(topicQueueSize, qos));
		cloudSub_4_.subscribe(this, "cloud4", RCLCPP_QOS(topicQueueSize, qos));
		if(approx)
		{
			approxSync4_ = new message_filters::Synchronizer<ApproxSync4Policy>(ApproxSync4Policy(syncQueueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_, cloudSub_4_);
			if(approxSyncMaxInterval > 0.0)
				approxSync4_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
			if(agePenalty > 0.0)
				approxSync4_->setAgePenalty(agePenalty);
			if(interMessageLowerBound > 0.0)
			{
				approxSync4_->setInterMessageLowerBound(0, rclcpp::Duration::from_seconds(interMessageLowerBound));
				approxSync4_->setInterMessageLowerBound(1, rclcpp::Duration::from_seconds(interMessageLowerBound));
				approxSync4_->setInterMessageLowerBound(2, rclcpp::Duration::from_seconds(interMessageLowerBound));
				approxSync4_->setInterMessageLowerBound(3, rclcpp::Duration::from_seconds(interMessageLowerBound));
			}
			approxSync4_->registerCallback(std::bind(&rtabmap_util::PointCloudAggregator::clouds4_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		}
		else
		{
			exactSync4_ = new message_filters::Synchronizer<ExactSync4Policy>(ExactSync4Policy(syncQueueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_, cloudSub_4_);
			exactSync4_->registerCallback(std::bind(&rtabmap_util::PointCloudAggregator::clouds4_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		}
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync%s):\n   %s,\n   %s,\n   %s,\n   %s",
				get_name(),
				approx?"approx":"exact",
				approx&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
				cloudSub_1_.getSubscriber()->get_topic_name(),
				cloudSub_2_.getSubscriber()->get_topic_name(),
				cloudSub_3_.getSubscriber()->get_topic_name(),
				cloudSub_4_.getSubscriber()->get_topic_name());
	}
	else if(count == 3)
	{
		cloudSub_3_.subscribe(this, "cloud3", RCLCPP_QOS(topicQueueSize, qos));
		if(approx)
		{
			approxSync3_ = new message_filters::Synchronizer<ApproxSync3Policy>(ApproxSync3Policy(syncQueueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_);
			if(approxSyncMaxInterval > 0.0)
				approxSync3_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
			if(agePenalty > 0.0)
				approxSync3_->setAgePenalty(agePenalty);
			if(interMessageLowerBound > 0.0)
			{
				approxSync3_->setInterMessageLowerBound(0, rclcpp::Duration::from_seconds(interMessageLowerBound));
				approxSync3_->setInterMessageLowerBound(1, rclcpp::Duration::from_seconds(interMessageLowerBound));
				approxSync3_->setInterMessageLowerBound(2, rclcpp::Duration::from_seconds(interMessageLowerBound));
			}
			approxSync3_->registerCallback(std::bind(&rtabmap_util::PointCloudAggregator::clouds3_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
		}
		else
		{
			exactSync3_ = new message_filters::Synchronizer<ExactSync3Policy>(ExactSync3Policy(syncQueueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_);
			exactSync3_->registerCallback(std::bind(&rtabmap_util::PointCloudAggregator::clouds3_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
		}
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync%s):\n   %s,\n   %s,\n   %s",
				this->get_name(),
				approx?"approx":"exact",
				approx&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
				cloudSub_1_.getSubscriber()->get_topic_name(),
				cloudSub_2_.getSubscriber()->get_topic_name(),
				cloudSub_3_.getSubscriber()->get_topic_name());
	}
	else
	{
		if(approx)
		{
			approxSync2_ = new message_filters::Synchronizer<ApproxSync2Policy>(ApproxSync2Policy(syncQueueSize), cloudSub_1_, cloudSub_2_);
			if(approxSyncMaxInterval > 0.0)
				approxSync2_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
			if(agePenalty > 0.0)
				approxSync2_->setAgePenalty(agePenalty);
			if(interMessageLowerBound > 0.0)
			{
				approxSync2_->setInterMessageLowerBound(0, rclcpp::Duration::from_seconds(interMessageLowerBound));
				approxSync2_->setInterMessageLowerBound(1, rclcpp::Duration::from_seconds(interMessageLowerBound));
			}
			approxSync2_->registerCallback(std::bind(&rtabmap_util::PointCloudAggregator::clouds2_callback, this, std::placeholders::_1, std::placeholders::_2));
		}
		else
		{
			exactSync2_ = new message_filters::Synchronizer<ExactSync2Policy>(ExactSync2Policy(syncQueueSize), cloudSub_1_, cloudSub_2_);
			exactSync2_->registerCallback(std::bind(&rtabmap_util::PointCloudAggregator::clouds2_callback, this, std::placeholders::_1, std::placeholders::_2));
		}
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync%s):\n   %s,\n   %s",
				this->get_name(),
				approx?"approx":"exact",
				approx&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
				cloudSub_1_.getSubscriber()->get_topic_name(),
				cloudSub_2_.getSubscriber()->get_topic_name());
	}


	warningThread_ = new std::thread([&](){
		rclcpp::Rate r(1.0/5.0);
		while(!callbackCalled_)
		{
			r.sleep();
			if(!callbackCalled_)
			{
				RCLCPP_WARN(this->get_logger(), "%s: Did not receive data since 5 seconds! Make sure the input topics are "
						"published (\"$ ros2 topic hz my_topic\") and the timestamps in their "
						"header are set. %s%s",
						this->get_name(),
						approx?"":"Parameter \"approx_sync\" is false, which means that input "
							"topics should have all the exact timestamp for the callback to be called.",
						subscribedTopicsMsg_.c_str());
			}
		}
	});
	RCLCPP_INFO(this->get_logger(), "%s", subscribedTopicsMsg_.c_str());
}

PointCloudAggregator::~PointCloudAggregator()
{
	delete exactSync4_;
	delete approxSync4_;
	delete exactSync3_;
	delete approxSync3_;
	delete exactSync2_;
	delete approxSync2_;

	if(warningThread_)
	{
		callbackCalled_=true;
		warningThread_->join();
		delete warningThread_;
	}
}

void PointCloudAggregator::clouds4_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_1,
					 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_2,
					 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_3,
					 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_4)
{
	std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds;
	clouds.push_back(cloudMsg_1);
	clouds.push_back(cloudMsg_2);
	clouds.push_back(cloudMsg_3);
	clouds.push_back(cloudMsg_4);

	combineClouds(clouds);
}
void PointCloudAggregator::clouds3_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_1,
					 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_2,
					 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_3)
{
	std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds;
	clouds.push_back(cloudMsg_1);
	clouds.push_back(cloudMsg_2);
	clouds.push_back(cloudMsg_3);

	combineClouds(clouds);
}
void PointCloudAggregator::clouds2_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_1,
					 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_2)
{
	std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds;
	clouds.push_back(cloudMsg_1);
	clouds.push_back(cloudMsg_2);

	combineClouds(clouds);
}
void PointCloudAggregator::combineClouds(const std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> & cloudMsgs) {
	callbackCalled_ = true;

	// pass if there are no subscribers
	if (!(convertToLaserScan_ ? scanPub_->get_subscription_count() : cloudPub_->get_subscription_count())) {
		return;
	}

	// select latest stamp: this will be the common temporal frame
	rclcpp::Time latest_stamp = cloudMsgs.front()->header.stamp;
	uint8_t latest_stamp_idx = 0;
	for (uint8_t i = 1; i < cloudMsgs.size(); ++i) {
		const rclcpp::Time this_stamp = cloudMsgs.at(i)->header.stamp;
		if (this_stamp > latest_stamp) {
			latest_stamp_idx = i;
			latest_stamp = this_stamp;
		}
	}

	const auto& base_frame = frameId_;

	pcl::PCLPointCloud2::Ptr output_cloud(new pcl::PCLPointCloud2);
	for (uint8_t i = 0; i < cloudMsgs.size(); ++i) {
		const auto& cloud = cloudMsgs.at(i);

		// spatial transform from sensor frame to base frame (static tf, commonly sensor->base_link. This should be always available)
		const rtabmap::Transform static_t = rtabmap_conversions::getTransform(base_frame, cloud->header.frame_id, cloud->header.stamp, *tfBuffer_, waitForTransform_);
		if (static_t.isNull()) {
			RCLCPP_ERROR(this->get_logger(), "Failed to get static transform during point cloud aggregation!");
			return;
		}
		// spatiotemporal transform from base frame to unified frame(at the timestamp of latest cloud)
		rtabmap::Transform spatiotemporal_t;
		spatiotemporal_t.setIdentity();
		if (i != latest_stamp_idx) {
			spatiotemporal_t = rtabmap_conversions::getMovingTransform(
				base_frame,
				fixedFrameId_,
				latest_stamp,
				cloud->header.stamp,
				*tfBuffer_,
				waitForTransform_);
			if (spatiotemporal_t.isNull()) {
				RCLCPP_WARN(this->get_logger(), "Failed to get moving transform during point cloud aggregation! Ignoring temporal shift...");
				spatiotemporal_t.setIdentity();
			}
		}
		// combine transforms to shift this cloud to common spatiotemporal frame
		const Eigen::Matrix4f combined_t = spatiotemporal_t.toEigen4f() * static_t.toEigen4f();
		sensor_msgs::msg::PointCloud2 tmp;
		rtabmap_conversions::transformPointCloudLite(combined_t, *cloud, tmp);
		// convert to PCL pointcloud2
		pcl::PCLPointCloud2::Ptr tmp_cloud(new pcl::PCLPointCloud2);
		pcl_conversions::toPCL(tmp, *tmp_cloud);

		// remove invalid pts
		if (!tmp_cloud->is_dense) {
			tmp_cloud = rtabmap::util3d::removeNaNFromPointCloud(tmp_cloud);
		}

		// strip fields other than xyz if xyzOutput_ is true
		if (xyzOutput_ and !tmp_cloud->data.empty()) {
			// convert only if not already XYZ cloud
			bool hasField[4] = {false};
			for(uint8_t i = 0; i < tmp_cloud->fields.size(); ++i)
			{
				if(tmp_cloud->fields[i].name.compare("x") == 0)
				{
					hasField[0] = true;
				}
				else if(tmp_cloud->fields[i].name.compare("y") == 0)
				{
					hasField[1] = true;
				}
				else if(tmp_cloud->fields[i].name.compare("z") == 0)
				{
					hasField[2] = true;
				}
				else
				{
					hasField[3] = true; // others (intensity, timestamp...)
					break;
				}
			}
			if(hasField[0] && hasField[1] && hasField[2] && !hasField[3])
			{
				// do nothing, already XYZ
			}
			else
			{
				pcl::PointCloud<pcl::PointXYZ> cloudxyz;
				pcl::fromPCLPointCloud2(*tmp_cloud, cloudxyz);
				pcl::toPCLPointCloud2(cloudxyz, *tmp_cloud);
			}
		}

		if (output_cloud->data.empty()) {
			output_cloud = tmp_cloud; // pointers
		} else if (!tmp_cloud->data.empty()) {
			if (output_cloud->fields.size() != tmp_cloud->fields.size()) {
				RCLCPP_WARN_ONCE(this->get_logger(), "Detected different fields for input clouds during point cloud aggregation! Please check formats...");
			}
			pcl::PCLPointCloud2::Ptr tmp_output_cloud(new pcl::PCLPointCloud2);
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
			pcl::concatenate(*output_cloud, *tmp_cloud, *tmp_output_cloud);
#else
			pcl::concatenatePointCloud(*output_cloud, *tmp_cloud, *tmp_output_cloud);
#endif
			tmp_output_cloud->row_step = tmp_output_cloud->width * tmp_output_cloud->point_step;
			output_cloud = tmp_output_cloud; // pointers
		} else {
			// RCLCPP_ERROR(this->get_logger(), "Something went wrong while concatenation...");
		}

	}

	// convert back to ROS cloud
	sensor_msgs::msg::PointCloud2::UniquePtr rosCloud(new sensor_msgs::msg::PointCloud2);
	pcl_conversions::moveFromPCL(*output_cloud, *rosCloud);
	rosCloud->header.stamp = latest_stamp;
	rosCloud->header.frame_id = base_frame;

	// convert pointcloud2 to laserscan, inspired by the pointcloud_to_laserscan package
	if (convertToLaserScan_) {
		sensor_msgs::msg::LaserScan::UniquePtr scanMsg(new sensor_msgs::msg::LaserScan);
		scanMsg->header = rosCloud->header;
		scanMsg->angle_min = scanAngleMin_;
		scanMsg->angle_max = scanAngleMax_;
		scanMsg->angle_increment = scanAngleIncrement_;
		scanMsg->time_increment = 0.0;
		scanMsg->scan_time = 0.0;
		scanMsg->range_min = scanRangeMin_;
		scanMsg->range_max = scanRangeMax_;
		scanMsg->ranges.assign(numPoints_, 0.0);

		sensor_msgs::PointCloud2ConstIterator<float> iterX(*rosCloud, "x");
		sensor_msgs::PointCloud2ConstIterator<float> iterY(*rosCloud, "y");
		// sensor_msgs::PointCloud2ConstIterator<float> iterZ(*rosCloud, "z");
		for (; iterX != iterX.end(); ++iterX, ++iterY/*, ++iterZ*/) {
			const float x = *iterX;
			const float y = *iterY;
			// const float z = *iterZ;
			const double range = std::hypot(x, y);
			if (range >= scanRangeMin_ and range <= scanRangeMax_) {
				const double angle = std::atan2(y, x);
				if (angle >= scanAngleMin_ and angle <= scanAngleMax_) {
					const uint32_t idx = (angle - scanAngleMin_) / scanAngleIncrement_;
					if (idx < numPoints_ and (range < scanMsg->ranges[idx] or scanMsg->ranges[idx] < 0.01)) {
						scanMsg->ranges[idx] = range;
					}
				}
			}
		}

		scanPub_->publish(std::move(scanMsg));
	} else {
		cloudPub_->publish(std::move(rosCloud));
	}
}
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_util::PointCloudAggregator)
