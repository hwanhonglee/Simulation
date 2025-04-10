
#include "carla_pointcloud/carla_pointcloud_interface_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
// #include <awf_velodyne_pointcloud/
#include <yaml-cpp/yaml.h>
#include <velodyne_pointcloud/func.h>
#include <velodyne_pointcloud/rawdata.h>
#include <pcl_ros/transforms.hpp>

//#include <pcl/io/io.h>
#include <memory>

// HH_250331
void PointCloudInterface::processScan(const sensor_msgs::msg::PointCloud2::SharedPtr scanMsg)
{
	if (b_create_ex_)
	{
		// Previous pointcloud parsing (PointXYZI)
		pcl::PointCloud<velodyne_pointcloud::PointXYZIRCAEDT>::Ptr input_cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIRCAEDT>);
		pcl::fromROSMsg(*scanMsg, *input_cloud);

		/*
		pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(*scanMsg, *input_cloud);

		Create PointXYZIRCAEDT Cloud
		pcl::PointCloud<velodyne_pointcloud::PointXYZIRCAEDT>::Ptr extended_cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIRCAEDT>);

		for (const auto& p : input_cloud->points)
		{
			velodyne_pointcloud::PointXYZIRCAEDT point;
			point.x = p.x;
			point.y = p.y;
			point.z = p.z;
			point.intensity = p.intensity;
			// HH_250331
			point.return_type = 0;
			point.padding = 0;

			// cal extend info 
			float d = 0;
			int ring = 0;
			int azimuth = 0;
			CalculatedExtendedCloudInformation(p, d, azimuth, ring);

			point.ring = static_cast<uint16_t>(ring);
			point.azimuth = static_cast<float>(azimuth);
			point.distance = d;
			point.time_stamp = rclcpp::Time(scanMsg->header.stamp).seconds();

			extended_cloud->points.push_back(point);
		}

		extended_cloud->width = extended_cloud->points.size();
		extended_cloud->height = 1;
		extended_cloud->is_dense = true;
		pcl_conversions::toPCL(scanMsg->header, extended_cloud->header);

		// Publish Convert ROS Msg
		auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
		pcl::toROSMsg(*extended_cloud, *ros_pc_msg_ptr);
		ros_pc_msg_ptr->header.frame_id = "sensor_kit_base_link";
		*/
		
		input_cloud->width = input_cloud->points.size();
		input_cloud->height = 1;
		input_cloud->is_dense = true;

		auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
		pcl::toROSMsg(*input_cloud, *ros_pc_msg_ptr);
		ros_pc_msg_ptr->header.frame_id = "sensor_kit_base_link";

		velodyne_points_pub_->publish(std::move(ros_pc_msg_ptr));
	}
	else
	{
		// Trasform Publish Based on TF (origin)
		sensor_msgs::msg::PointCloud2 transformed_cloud;
		if (pcl_ros::transformPointCloud(tf_output_frame_, *scanMsg, transformed_cloud, *tf_buffer_))
		{
			transformed_cloud.header.stamp = scanMsg->header.stamp;
			velodyne_points_localization->publish(transformed_cloud);
			velodyne_points_perception->publish(transformed_cloud);
		}
	}
}


void PointCloudInterface::CalculatedExtendedCloudInformation(const pcl::PointXYZI& point, float& distance, int& azimuth, int& ring_id)
{
	static float tmp[32]={-30.67,-29.33,-28.00, -26.66,-25.33,-24.00,-22.67,-21.33,-20.00,
			-18.67,-17.33,-16.00,-14.67,-13.33,-12.00,-10.67,-9.33,-8.00,
			-6.66,-5.33,-4.00,-2.67,-1.33,0.00,1.33,2.67,4.00,5.33,6.67,8.00,
			9.33,10.67};
	std::vector<float> angle_v_beam = std::vector<float>(tmp, tmp + 32);	// vertical angle of beams

	/* Calculate beam id of each point */
	float xy = std::sqrt(point.x * point.x + point.y * point.y);
	float r = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
	float angle_v = std::abs(std::acos(xy / r) / M_PI * 180.0);	// vertical angle of the point

	if(point.z < 0)
	{
		angle_v =- angle_v;
	}

	int beam_id = 0;
	float angle_v_min = std::abs(angle_v - angle_v_beam.front());
	for(unsigned int id = 1; id < angle_v_beam.size(); id++)
	{
		float d = std::abs(angle_v - angle_v_beam[id]);
		if(d < angle_v_min)
		{
			angle_v_min = d;
			beam_id = id;
		}
	}


	azimuth = std::atan2(point.y,point.x)/M_PI*180.0;
	ring_id = beam_id;
//	float xz = std::sqrt((point.y * point.y) + (point.z * point.z));
	distance = xy;
}

void PointCloudInterface::setupTF()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

PointCloudInterface::~PointCloudInterface()
{

}

PointCloudInterface::PointCloudInterface(const rclcpp::NodeOptions & node_options)
: Node("carla_pointcloud_interface_node", node_options), tf_output_frame_("base_link"), b_create_ex_(false)
{

	carla_cloud_ =
		    this->create_subscription<sensor_msgs::msg::PointCloud2>(
		    "carla_pointcloud", rclcpp::SensorDataQoS(),
		    std::bind(&PointCloudInterface::processScan, this, std::placeholders::_1));

//	auto_control_cmd =
//		    this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
//		    "/control/command/control_cmd", rclcpp::SensorDataQoS(),
//		    std::bind(&PointCloudInterface::onAutoCtrlCmd, this, std::placeholders::_1));
//  rclcpp::QoS durable_qos{1};
//  durable_qos.transient_local();
//	control_pub = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("carla_interface_control_cmd", 1);


	if(b_create_ex_)
	{
	  velodyne_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/top/pointcloud_raw", rclcpp::SensorDataQoS());
	  velodyne_points_ex_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/top/pointcloud_raw_ex", rclcpp::SensorDataQoS());
	  velodyne_points_combined_ex_pub_ =
	    this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/top/pointcloud_combined_ex", rclcpp::SensorDataQoS());
	}
	else
	{
		setupTF();
		velodyne_points_localization =
			    this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/top/outlier_filtered/pointcloud", rclcpp::SensorDataQoS());
		velodyne_points_perception =
			    this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/concatenated/pointcloud", rclcpp::SensorDataQoS());
	}
	
}
// }

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudInterface)
