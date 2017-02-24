#include <ros/ros.h>

#include <marker_msgs/MarkerStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <string>


void loadParam(ros::NodeHandle &n, const std::string &str, bool &param) {
	if( !n.getParam( str, param ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: %s", str.c_str(), param ? "true" : "false" );
	} else {
		ROS_INFO( "Loaded %s: %s", str.c_str(), param ? "true" : "false" );
	}
}

void loadParam(ros::NodeHandle &n, const std::string &str, int &param) {
	if( !n.getParam( str, param ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: %i", str.c_str(), param );
	} else {
		ROS_INFO( "Loaded %s: %i", str.c_str(), param );
	}
}

void loadParam(ros::NodeHandle &n, const std::string &str, double &param) {
	if( !n.getParam( str, param ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: %f", str.c_str(), param );
	} else {
		ROS_INFO( "Loaded %s: %f", str.c_str(), param );
	}
}

void loadParam(ros::NodeHandle &n, const std::string &str, std::string &param) {
	if( !n.getParam( str, param ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: %s", str.c_str(), param.c_str() );
	} else {
		ROS_INFO( "Loaded %s: %s", str.c_str(), param.c_str() );
	}
}

class MarkerLandmarks {
	private:
		ros::NodeHandle nh_;
		ros::Subscriber marker_sub_;

		ros::Time last_stamp;

		bool use_lpf;
		double lpf_beta;

		double min_confidence;

		tf::Vector3 first_lm_pos;

		std::vector< int > known_landmarks;
		std::vector< std::vector< int > > landmarks_list;

	public:
		MarkerLandmarks() : nh_(ros::this_node::getName()), last_stamp(0) {
			std::string marker_topic = "markers";

			loadParam(nh_, "marker_topic", marker_topic);
			loadParam(nh_, "use_lpf", use_lpf);
			loadParam(nh_, "lpf_beta", lpf_beta);
			loadParam(nh_, "min_confidence", min_confidence);

			marker_sub_ = nh_.subscribe<marker_msgs::MarkerStamped> ( marker_topic, 100, &MarkerLandmarks::marker_cb, this );

			ROS_INFO("Listening for new markers...");
		}

		~MarkerLandmarks() {
		}

		void marker_cb(const marker_msgs::MarkerStamped::ConstPtr& msg) {

		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "ml_landmarks");
	MarkerLandmarks ml;

	ros::spin();
	/*
	while(ros::ok()) {
		ros::spinOnce();
		ml.sleep()
	}
	*/
	return 0;
}
