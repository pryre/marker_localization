#include <ros/ros.h>

#include <ml_msgs/MarkerDetection.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

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

	param = tf2::Vector3( temp_vec.at(0), temp_vec.at(1), temp_vec.at(2) );
}

void loadParam(ros::NodeHandle &n, const std::string &str, tf2::Quaternion &param) {
	std::vector< double > temp_vec;

	if( !n.getParam( str, temp_vec ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: [%f, %f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2), temp_vec.at(3) );
	} else {
		ROS_INFO( "Loaded %s: [%f, %f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2), temp_vec.at(3) );
	}

	param = tf2::Quaternion( temp_vec.at(1), temp_vec.at(2), temp_vec.at(3), temp_vec.at(0) );
}

void loadParam(ros::NodeHandle &n, const std::string &str, geometry_msgs::Transform &param) {
	std::vector< double > temp_vec = {0,0,0,1,0,0,0};

	if( !n.getParam( str, temp_vec ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: [%f, %f, %f], [%f, %f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2), temp_vec.at(3), temp_vec.at(4), temp_vec.at(5), temp_vec.at(6) );
	} else {
		ROS_INFO( "Loaded %s: [%f, %f, %f], [%f, %f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2), temp_vec.at(3), temp_vec.at(4), temp_vec.at(5), temp_vec.at(6) );
	}

	param.translation.x = temp_vec.at(0);
	param.translation.y = temp_vec.at(1);
	param.translation.z = temp_vec.at(2);
	param.rotation.w = temp_vec.at(3);
	param.rotation.x = temp_vec.at(4);
	param.rotation.y = temp_vec.at(5);
	param.rotation.z = temp_vec.at(6);
}

void poseToTransform( const geometry_msgs::Pose &p, geometry_msgs::Transform &t ) {
	t.translation.x = p.position.x;
	t.translation.y = p.position.y;
	t.translation.z = p.position.z;

	t.rotation.w = p.orientation.w;
	t.rotation.x = p.orientation.x;
	t.rotation.y = p.orientation.y;
	t.rotation.z = p.orientation.z;
}

void vector3MsgToTF2( const geometry_msgs::Vector3 &vg, tf2::Vector3 &vt ) {
	vt = tf2::Vector3( vg.x, vg.y, vg.z );
}

void vector3TF2ToMsg( const tf2::Vector3 &vt, geometry_msgs::Vector3 &vg ) {
	vg.x = vt.x();
	vg.y = vt.y();
	vg.z = vt.z();
}

void quaternionMsgToTF2( const geometry_msgs::Quaternion &qg, tf2::Quaternion &qt ) {
	qt = tf2::Quaternion (qg.x, qg.y, qg.z, qg.w);
}

void quaternionTF2ToMsg( const tf2::Quaternion &qt, geometry_msgs::Quaternion &qg  ) {
	qg.x = qt.x();
	qg.y = qt.y();
	qg.z = qt.z();
	qg.w = qt.w();
}

void transformMsgToTF2( const geometry_msgs::Transform &g, tf2::Transform &t ) {
	tf2::Vector3 v;
	tf2::Quaternion q;

	vector3MsgToTF2( g.translation, v);
	t.setOrigin(v);

	quaternionMsgToTF2( g.rotation, q );
	t.setRotation(q);
}

void transformTF2ToMsg( const tf2::Transform &t, geometry_msgs::Transform &g ) {
	vector3TF2ToMsg( t.getOrigin(), g.translation );
	quaternionTF2ToMsg( t.getRotation(), g.rotation );
}

class MarkerLandmarks {
	private:
		ros::NodeHandle nh_;
		ros::Subscriber marker_sub_;
		ros::Publisher pose_pub_;

		std::string map_frame;
		int lm_static_ref_id;
		geometry_msgs::Transform lm_static_ref_pos;

		double min_confidence;
		bool snap_to_ref_plane;
		bool lpf_pos_apply;
		double lpf_pos_beta;

		bool found_static_ref;
		std::vector< int > known_landmarks;
		std::vector< std::vector< int > > landmarks_list;

		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfln_;
		tf2_ros::TransformBroadcaster tfbr_;
		tf2_ros::StaticTransformBroadcaster tfbrs_;

	public:
		MarkerLandmarks() : nh_(ros::this_node::getName()), tfln_(tfBuffer) {
			std::string topic_marker_detected = "marker_detection";
			std::string topic_pose_estimate = "camera_pose";
			map_frame = "map";
			found_static_ref = false;
			lm_static_ref_id = -1;

			loadParam(nh_, "topic_marker_detected", topic_marker_detected);
			loadParam(nh_, "topic_pose_estimate", topic_pose_estimate);

			loadParam(nh_, "map_frame", map_frame);
			loadParam(nh_, "lm_static_ref_id", lm_static_ref_id);
			loadParam(nh_, "lm_static_ref_pose", lm_static_ref_pos);

			if( lm_static_ref_id >= 0 ) {
				//Set the known lists to reflect it
				known_landmarks.push_back(lm_static_ref_id);

				std::vector< int > temp_list;	//This creates a list of 1, as it is the first node in the branch
				temp_list.push_back(lm_static_ref_id);
				landmarks_list.push_back(temp_list);

				geometry_msgs::TransformStamped tf_static_ref;
				tf_static_ref.header.stamp = ros::Time::now();
				tf_static_ref.header.frame_id = map_frame;
				tf_static_ref.child_frame_id = "ml/id_" + std::to_string(lm_static_ref_id);
				tf_static_ref.transform = lm_static_ref_pos;

				tfbrs_.sendTransform(tf_static_ref);	//Send off a once-off transform to define the start point
			}

			loadParam(nh_, "min_confidence", min_confidence);
			loadParam(nh_, "snap_to_ref_plane", snap_to_ref_plane);
			loadParam(nh_, "lpf_pos_apply", lpf_pos_apply);
			if (lpf_pos_apply)	//Cut down on warning messages
				loadParam(nh_, "lpf_pos_beta", lpf_pos_beta);

			marker_sub_ = nh_.subscribe<ml_msgs::MarkerDetection> ( topic_marker_detected, 100, &MarkerLandmarks::marker_cb, this );
			pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ( topic_pose_estimate, 100 );

			ROS_INFO("Listening for new markers...");
		}

		~MarkerLandmarks() {
		}

		void marker_cb(const ml_msgs::MarkerDetection::ConstPtr& msg) {
			std::vector< int > found_landmarks;			//List of indexes for msg.markers[] that represent the new landmarks to be added to the TF
			std::vector< int > found_reference_points;	//List of indexes for msg.markers[] that represent known reference points
			std::vector< int > known_reference_points;	//List of indexes for known_landmarks that represent known reference points

			for(int i = 0; i < msg->markers.size(); i++) {	//Sort through the list of detected markers
				if(msg->markers.at(i).marker_confidence >= min_confidence) {
					//Search the list of known landmarks to see if we know of this marker
					std::vector<int>::iterator it = std::find( known_landmarks.begin(), known_landmarks.end(), msg->markers.at(i).marker_id );

					//If we know it
					if( it != known_landmarks.end() ) {	//A match was found in the image for id of the board
						int ind = std::distance( known_landmarks.begin(), it);	//Get the index of the itterator

						//Add it to the list of references
						known_reference_points.push_back(ind);
						found_reference_points.push_back(i);

					} else if( lm_static_ref_id < 0 )  {	//Take the first marker found to be the static ref
							lm_static_ref_id = msg->markers.at(i).marker_id;	//Set the id as known if to cover the "any marker first" case

							ROS_INFO("Found the static reference marker: %i", lm_static_ref_id);

							known_landmarks.push_back(lm_static_ref_id);	//Set the known lists to reflect it

							std::vector< int > temp_list;	//This creates a list of 1, as it is the first node in the branch
							temp_list.push_back(lm_static_ref_id);
							landmarks_list.push_back(temp_list);

							//Add it to the list of references
							known_reference_points.push_back(known_landmarks.size() - 1);
							found_reference_points.push_back(i);

							//Send off a once-off transform to define the start point
							geometry_msgs::TransformStamped tf;
							tf.header.stamp = msg->header.stamp;
							tf.header.frame_id = map_frame;
							tf.child_frame_id = "ml/id_" + std::to_string(lm_static_ref_id);
							tf.transform = lm_static_ref_pos;
							tfbrs_.sendTransform(tf);
					} else {
						found_landmarks.push_back(i);	//Add the new landmarks to the shortlist
					}
				}
			}
			//If at least 1 known reference was found
			if( known_reference_points.size() > 0 ) {
				//Check the landmark list at each reference to find the closest branch to the starting reference (TODO: This could be simplified if known_landmarks(i) represents the same marker as landmarks_list(i))
				int ind_ref = known_reference_points.at(0);
				int marker_ref = found_reference_points.at(0);

				//Sort for the best reference marker out of the ones detected
				for(int j = 1; j < known_reference_points.size(); j++) { //Start by comparing ind 1 to ind 0
					//XXX: Here we sort priority by confidence first, then by branch size
					//TODO: May be good to also consider the marker closest to the camera
					if( msg->markers.at(found_reference_points.at(j)).marker_confidence > msg->markers.at(marker_ref).marker_confidence ) {
						ind_ref = known_reference_points.at(j);
						marker_ref = found_reference_points.at(j);
					} else if( msg->markers.at(found_reference_points.at(j)).marker_confidence == msg->markers.at(marker_ref).marker_confidence ) {
						if( landmarks_list.at(known_reference_points.at(j)).size() < landmarks_list.at(ind_ref).size()) {
							ind_ref = known_reference_points.at(j);
							marker_ref = found_reference_points.at(j);
						}
					}
				}

				//Estimate the pose of the camera from the reference marker
				geometry_msgs::TransformStamped gt_ref;
				tf2::Transform tf_ref;
				gt_ref.header.stamp = msg->header.stamp;
				gt_ref.header.frame_id = "ml/id_" + std::to_string(msg->markers.at(marker_ref).marker_id);
				gt_ref.child_frame_id = "ml/" + msg->header.frame_id;

				poseToTransform(msg->markers.at(marker_ref).pose, gt_ref.transform);
				transformMsgToTF2(gt_ref.transform, tf_ref);
				transformTF2ToMsg(tf_ref.inverse(), gt_ref.transform);
				tfbr_.sendTransform(gt_ref);

				//For all the already known markers
				for(int k = 0; k < known_reference_points.size(); k++) {
					//For all execpt the best reference point and the static landmark
					if( ( known_reference_points.at(k) != ind_ref) && (known_landmarks.at(known_reference_points.at(k)) != lm_static_ref_id) ) {
						bool update_transform = false;
						tf2::Transform lm_tf;

						std::vector< int > found_branch = landmarks_list.at(known_reference_points.at(k));	//The refernce branch of the current marker

						//If this reference point is a closer branch
						if( landmarks_list.at(ind_ref).size() < ( found_branch.size() - 1 ) ) {
							//Relate the marker to the new closest branch
							int id_new = known_landmarks.at(known_reference_points.at(k));
							std::vector< int > temp_list;
							temp_list = landmarks_list.at(ind_ref);	//Get the branch for the index reference
							temp_list.push_back(id_new);	//Append the new id
							landmarks_list.at(known_reference_points.at(k)) = temp_list;	//Add this to the known lists

							//Send out a message to notify of a moving branch
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

							update_transform = true;
						}	//Else it's already in a equal or better branch

						//Broadcast the new transform for this marker
						if(update_transform) {
							if(snap_to_ref_plane) {
								//TODO: Align to axis
							}

							//TODO: Actually work out the new transform
						}
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

					geometry_msgs::TransformStamped tf_temp;
					tf_temp.header.stamp = msg->header.stamp;
					tf_temp.header.frame_id = "ml/" + msg->header.frame_id;
					tf_temp.child_frame_id = "ml/id_" + std::to_string(id_new);
					poseToTransform(msg->markers.at(found_landmarks.at(f)).pose, tf_temp.transform);

					tfBuffer.setTransform(tf_temp, "ml_temp_transform");	//Add a temporary transform to the local listener

					//Broadcast the new transform for this marker
					try {
						geometry_msgs::TransformStamped tf_new;
						tf_new = tfBuffer.lookupTransform( "ml/id_" + std::to_string(msg->markers.at(marker_ref).marker_id), "ml/id_" + std::to_string(id_new), msg->header.stamp, ros::Duration(0.1) );

						if(snap_to_ref_plane) {
							tf_new.transform.translation.z = 0;

							tf2::Quaternion q;
							quaternionMsgToTF2( tf_new.transform.rotation, q );
							tf2::Matrix3x3 r( q );
							tf2::Vector3 body_x;
							tf2::Vector3 body_y( r.getRow(1) );
							tf2::Vector3 body_z(0.0, 0.0, 1.0);

							body_x = body_y.cross(body_z);
							body_x.normalize();
							body_y = body_z.cross(body_x);
							r.setValue( body_x.x(), body_x.y(), body_x.z(), body_y.x(), body_y.y(), body_y.z(), body_z.x(), body_z.y(), body_z.z() );

							r.getRotation( q );
							quaternionTF2ToMsg(q, tf_new.transform.rotation);
						}

						tfbrs_.sendTransform(tf_new);
					} catch (tf2::TransformException &ex) {
						ROS_WARN("%s",ex.what());
						ros::Duration(1.0).sleep();
						continue;
					}
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
