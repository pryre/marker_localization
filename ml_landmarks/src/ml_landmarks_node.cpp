#include <ros/ros.h>

#include <ml_msgs/MarkerDetection.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
//#include <tf/transform_listener.h>
//#include <tf/transform_broadcaster.h>

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

void loadParam(ros::NodeHandle &n, const std::string &str, tf2::Vector3 &param) {
	std::vector< double > temp_vec;

	if( !n.getParam( str, temp_vec ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: [%f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2) );
	} else {
		ROS_INFO( "Loaded %s: [%f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2) );
	}

	param.setX( temp_vec.at(0) );
	param.setY( temp_vec.at(1) );
	param.setZ( temp_vec.at(2) );
}

class MarkerLandmarks {
	private:
		ros::NodeHandle nh_;
		ros::Subscriber marker_sub_;
		ros::Publisher pose_pub_;

		std::string map_frame;
		int lm_static_ref_id;
		tf2::Vector3 lm_static_ref_pos;

		double min_confidence;
		bool snap_quat_apply;
		tf2::Vector3 snap_quat_plane;
		bool lpf_pos_apply;
		double lpf_pos_beta;

		bool found_static_ref;
		std::vector< int > known_landmarks;
		std::vector< std::vector< int > > landmarks_list;

	public:
		MarkerLandmarks() : nh_(ros::this_node::getName()) {
			std::string topic_marker_detected = "marker_detection";
			std::string topic_pose_estimate = "camera_pose";
			map_frame = "map";
			found_static_ref = false;
			lm_static_ref_id = -1;

			loadParam(nh_, "topic_marker_detected", topic_marker_detected);
			loadParam(nh_, "topic_pose_estimate", topic_pose_estimate);

			loadParam(nh_, "map_frame", map_frame);
			loadParam(nh_, "lm_static_ref_id", lm_static_ref_id);
			loadParam(nh_, "lm_static_ref_pos", lm_static_ref_pos);

			if( lm_static_ref_id >= 0 ) {
				//Set the known lists to reflect it
				known_landmarks.push_back(lm_static_ref_id);

				std::vector< int > temp_list;	//This creates a list of 1, as it is the first node in the branch
				temp_list.push_back(lm_static_ref_id);
				landmarks_list.push_back(temp_list);

				//TODO: Send off a once-off transform to define the start point
				//tfbr_.broadcast(map->ml/lm_static_ref_id);
			}

			loadParam(nh_, "min_confidence", min_confidence);
			loadParam(nh_, "snap_quat_apply", snap_quat_apply);
			if(snap_quat_apply)	//Cut down on warning messages
				loadParam(nh_, "snap_quat_plane", snap_quat_plane);
			loadParam(nh_, "lpf_pos_apply", lpf_pos_apply);
			if (lpf_pos_apply)	//Cut down on warning messages
				loadParam(nh_, "lpf_pos_beta", lpf_pos_beta);

			//TODO: Marker Vizualization stuff

			marker_sub_ = nh_.subscribe<ml_msgs::MarkerDetection> ( topic_marker_detected, 100, &MarkerLandmarks::marker_cb, this );
			pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ( topic_pose_estimate, 100 );

			ROS_INFO("Listening for new markers...");
		}

		~MarkerLandmarks() {
		}

		void marker_cb(const ml_msgs::MarkerDetection::ConstPtr& msg) {
			std::vector< int > found_landmarks;			//List of indexes for msg.markers[] that represent the new landmarks to be added to the TF
			std::vector< int > found_reference_points;	//List of indexes for known_landmarks that represent known reference points

			//TODO: CHECK FOR CONFIDENCE OF EACH MARKER AND COMPARE TO MIN IN THIS LOOP
			for(int i = 0; i < msg->markers.size(); i++) {	//Sort through the list of detected markers
				//Search the list of known landmarks to see if we know of this marker
				std::vector<int>::iterator it = std::find( known_landmarks.begin(), known_landmarks.end(), msg->markers.at(i).marker_id );

				//If we know it
				if( it != known_landmarks.end() ) {	//A match was found in the image for id of the board
					int ind = std::distance( known_landmarks.begin(), it);	//Get the index of the itterator

					//Add it to the list of references
					found_reference_points.push_back(ind);

				} else if( lm_static_ref_id < 0 )  {	//Take the first marker found to be the static ref
						lm_static_ref_id = msg->markers.at(i).marker_id;	//Set the id as known if to cover the "any marker first" case

						ROS_INFO("Found the static reference marker: %i", lm_static_ref_id);

						//Set the known lists to reflect it
						known_landmarks.push_back(lm_static_ref_id);

						std::vector< int > temp_list;	//This creates a list of 1, as it is the first node in the branch
						temp_list.push_back(lm_static_ref_id);
						landmarks_list.push_back(temp_list);

						//Add it to the list of references
						found_reference_points.push_back(known_landmarks.size() - 1);

						//TODO: Send off a once-off transform to define the start point
						//tfbr_.broadcast(map->ml/lm_static_ref_id);
				} else {
					//Add the new landmarks to the shortlist
					found_landmarks.push_back(i);
				}
			}
			//If at least 1 known reference was found
			if( found_reference_points.size() > 0 ) {
				//Check the landmark list at each reference to find the closest branch to the starting reference (TODO: This could be simplified if known_landmarks(i) represents the same marker as landmarks_list(i))
				int ind_ref = found_reference_points.at(0);

				//Go through the list and find the index that has the smallest branch list
				for(int j = 1; j < found_reference_points.size(); j++) { //Start by comparing ind 1 to ind 0
					if( landmarks_list.at(found_reference_points.at(j)).size() < landmarks_list.at(ind_ref).size())
						ind_ref = found_reference_points.at(j);
				}

				//ROS_INFO_THROTTLE(0.2, "Chosen reference id: %i", known_landmarks.at(ind_ref));

				//TODO:
				//	Loop through all the detected markers and push the temporary tramsforms in camera frame to TF
				//	Except for the ind_ref marker, which should align the camera in the ind_ref frame

				//TODO: broadcast here the transform from ind_ref -> cam

				//For all the already known markers
				for(int k = 0; k < found_reference_points.size(); k++) {
					//For all execpt the best reference point
					if(found_reference_points.at(k) != ind_ref) {
						bool update_transform = false;
						tf2::Transform lm_tf;

						std::vector< int > found_branch = landmarks_list.at(found_reference_points.at(k));	//The refernce branch of the current marker

						//If this reference point is a closer branch
						if( landmarks_list.at(ind_ref).size() < ( found_branch.size() - 1 ) ) {
							//Relate the marker to the new closest branch
							int id_new = known_landmarks.at(found_reference_points.at(k));
							std::vector< int > temp_list;
							temp_list = landmarks_list.at(ind_ref);	//Get the branch for the index reference
							temp_list.push_back(id_new);	//Append the new id
							landmarks_list.at(found_reference_points.at(k)) = temp_list;	//Add this to the known lists

							std::string temp_str = "Found shorter branch for id " + std::to_string(id_new) + ": [";
							for(int c = 0; c < temp_list.size(); c++) {
								if(c < temp_list.size() - 1) {
									temp_str += std::to_string( temp_list.at(c) ) + ",";
								} else {
									temp_str += std::to_string( temp_list.at(c) );
								}
							}
							temp_str += "]";
							ROS_INFO("%s", temp_str.c_str());

							update_transform = true;
						} else if ( (found_branch.at(found_branch.size() - 2) == ind_ref) && lpf_pos_apply) {	//Else if ind_ref is the current markers parent
							//TODO: Filter the position

							//TODO: update_transform = true;
						}	//Else it's already in a equal or better branch

						//Broadcast the new transform for this marker
						if(update_transform) {
							if(snap_quat_apply) {
								//TODO: Align to axis
							}

							//TODO: Actually work out the new transform
						}

						//If param set to do marker viz
							//TODO: Publish marker viz
					}
				}

				//For all of the new landmarks
				for(int f = 0; f < found_landmarks.size(); f++) {
					tf2::Transform lm_tf;
					int id_new = msg->markers.at(found_landmarks.at(f)).marker_id;

					//Set the known lists to reflect it
					known_landmarks.push_back(id_new);

					std::vector< int > temp_list;
					temp_list = landmarks_list.at(ind_ref);	//Get the branch for the index reference
					temp_list.push_back(id_new);	//Append the new id
					landmarks_list.push_back(temp_list);	//Add this to the known lists

					std::string temp_str = "New marker with id " + std::to_string(id_new) + ": [";
					for(int c = 0; c < temp_list.size(); c++) {
						if(c < temp_list.size() - 1) {
							temp_str += std::to_string( temp_list.at(c) ) + ",";
						} else {
							temp_str += std::to_string( temp_list.at(c) );
						}
					}
					temp_str += "]";
					ROS_INFO("%s", temp_str.c_str());

					if(snap_quat_apply) {
						//TODO: Align to axis
					}

					//TODO: Broadcast the new transform for this marker

					//If param set to do marker viz
						//TODO: Publish marker viz
				}
			}
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
