#include <detector_ros/detector_ros.hpp>

using namespace ariitk::detector_ros;

int main(int argc, char** argv) {
	ros::init(argc, argv, "detector_node");
	ros::NodeHandle nh;

	DetectorROS detect;

	detect.init(nh);

	ros::Rate loop_rate(20);

	while (ros::ok()) {
		detect.run();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
