#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

class PoseTF
{
public:
	PoseTF() :nh_("~")
	{
		nh_.param("src_frame_id", src_frame_id, std::string("/imu_pose"));
		nh_.param("des_frame_id", des_frame_id, std::string("/footprint"));
		nh_.param("src_data_tpc", src_data_tpc, std::string("/camera_pose"));
		nh_.param("des_data_tpc", des_data_tpc, std::string("/payload_pose"));
		registerTopic();
	};
	virtual ~PoseTF(){};

private:
	tf::TransformListener tf_listener_;
	ros::NodeHandle nh_;
	std::string src_frame_id, des_frame_id, src_data_tpc, des_data_tpc;
	ros::Subscriber sub_sensor_pose;
	ros::Publisher pub_footprint;

	void registerTopic()
	{
		pub_footprint = nh_.advertise<geometry_msgs::PoseStamped>(des_data_tpc, 1000);
		sub_sensor_pose = nh_.subscribe<nav_msgs::Odometry>(src_data_tpc, 1000, &PoseTF::poseTF_Callback, this);
	};
	void poseTF_Callback(const nav_msgs::OdometryConstPtr& odom_msg)
	{
		geometry_msgs::PoseStamped src_frame_pose,des_frame_pose_;
		src_frame_pose.pose = odom_msg->pose.pose;
		src_frame_pose.header.stamp = ros::Time();
		src_frame_pose.header.frame_id = src_frame_id;
		ROS_INFO_STREAM("src_frame_pose: "<<src_frame_pose);
		try
		{
			tf_listener_.transformPose(des_frame_id, src_frame_pose, des_frame_pose_);
			ROS_INFO_STREAM("des_frame_pose: "<<des_frame_pose_);
			pub_footprint.publish(des_frame_pose_);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what()); //Print exception which was caught
		}
	};
};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "Pub_Footprint");
	PoseTF i_poseTF;
	ros::spin();
	return 0;
}