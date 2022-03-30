//
// Created by cwy on 2022/3/28.
//

#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>

#include <sensor_msgs/PointCloud2.h>

#include "laser_mapping.h"

void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
	LOG(INFO) << "msg timestamp: " << msg->header.stamp;
}

auto compute_delta_rotation(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2) -> Eigen::Matrix3d {
	Eigen::Vector3d axis = n1.cross(n2);
	axis /= axis.norm();
	return Eigen::AngleAxisd(std::acos(n1.dot(n2) / (n1.norm() * n2.norm())), axis).toRotationMatrix();
}

int main(int argc, char** argv) {
	using namespace faster_lio::common;

	FLAGS_stderrthreshold  = google::INFO;
	FLAGS_colorlogtostderr = true;
	google::InitGoogleLogging(argv[0]);

	Eigen::Vector3d t_bt {0.172056, 0.005340, 0};

	Eigen::Matrix3d Rbt(Eigen::AngleAxisd(-1.57 + 0.001611, Eigen::Vector3d::UnitZ())
	                      * Eigen::AngleAxisd(-1.57, Eigen::Vector3d::UnitX()));


	std::cout << "Rbt: \n" << Rbt.matrix() << std::endl;

	Eigen::Vector3d normal_t {-0.009825, -0.999874, -0.012454};
	Eigen::Vector3d normal_b = Rbt * normal_t;

	std::cout << "normal_b: \n" << normal_b << std::endl;

	Eigen::Vector3d axis_z {0, 0, 1};

	auto delta_R = compute_delta_rotation(normal_b, axis_z); // Tbb'

	Eigen::Matrix3d Rbt_calib = delta_R * Rbt; // Tbt' = Tbb' * Tbt

	auto euler = Rbt_calib.eulerAngles(2, 1, 0);

	std::cout << "euler: \n" << euler << std::endl;

	Eigen::Isometry3d Tbt_calib(Rbt_calib);
	Tbt_calib.translation() = t_bt;

	std::cout << "Tbt_calib: \n" << Tbt_calib.matrix() << std::endl;

	Eigen::Vector3d axis_z_test = Rbt_calib * normal_t;

	std::cout << "axis_z_test: \n" << axis_z_test << std::endl;

//	ros::init(argc, argv, "faster_lio");
//	ros::NodeHandle nh;
//
//	ros::Subscriber sub = nh.subscribe("/pc_front", 1000, point_cloud_callback);
//
//	ros::spin();

	return 0;
}