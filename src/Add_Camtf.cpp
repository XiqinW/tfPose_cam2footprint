#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Add_Camtf");
	ros::NodeHandle n("~");
	{
		//get frame ids
		std::string src_frame_id, des_frame_id;
		n.param("src_frame_id", src_frame_id, std::string("/imu"));
		n.param("des_frame_id", des_frame_id, std::string("/cam"));
		//get transform: cam --> imu
		std::string config_file;
		n.param("config_file", config_file, std::string("/home/xiqin/catkin_ws/src/sensor2footprint/cfg/cam_on_xq/ex_matrix.yaml"));
		cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
		if(!fsSettings.isOpened())
		{
			std::cerr << "ERROR: Wrong path to settings" << std::endl;
		}
		cv::Mat cv_R, cv_T;
		fsSettings["extrinsicRotation"] >> cv_R;
		fsSettings["extrinsicTranslation"] >> cv_T;
		fsSettings.release();
		//data conversions
		Eigen::Matrix3d eigen_R;
		Eigen::Vector3d eigen_T;
		cv::cv2eigen(cv_R, eigen_R);
		cv::cv2eigen(cv_T, eigen_T);
		Eigen::Isometry3d eigen_Transform(eigen_R);
		eigen_Transform.pretranslate(eigen_T);
		//inverse transform: imu --> cam, cam as child of imu in tf_tree
		eigen_Transform = eigen_Transform.inverse();
		ROS_INFO_STREAM("Transform: from "+src_frame_id+" to "+des_frame_id << std::endl << eigen_Transform.matrix());
		//eigen -> tf
		tf::Pose tf_Transform;
		tf::poseEigenToTF(eigen_Transform, tf_Transform);
		//send transform
		static tf::TransformBroadcaster br;
		ros::Rate r(20);
		while (n.ok())
		{
			br.sendTransform( tf::StampedTransform(tf_Transform, ros::Time::now(),
											src_frame_id, des_frame_id)
							);
			r.sleep();
		}
	}
	return 0;
}