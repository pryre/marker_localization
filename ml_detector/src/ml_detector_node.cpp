#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <ml_msgs/Marker.h>
#include <ml_msgs/MarkerDetection.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <dynamic_reconfigure/server.h>
#include <ml_detector/DetectorParamsConfig.h>
#include <ml_detector/SystemParamsConfig.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <vector>
#include <string>
#include <map>

typedef enum {
	BC_ID = 0,
	BC_ROWS,
	BC_COLS
} board_config_names;

void pointMsgToTF2( const geometry_msgs::Point &vg, tf2::Vector3 &vt ) {
	vt = tf2::Vector3( vg.x, vg.y, vg.z );
}

void pointTF2ToMsg( const tf2::Vector3 &vt, geometry_msgs::Point &vg ) {
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

void poseMsgToTF2( const geometry_msgs::Pose &g, tf2::Transform &t ) {
	tf2::Vector3 v;
	tf2::Quaternion q;

	pointMsgToTF2( g.position, v);
	t.setOrigin(v);

	quaternionMsgToTF2( g.orientation, q );
	t.setRotation(q);
}

void poseTF2ToMsg( const tf2::Transform &t, geometry_msgs::Pose &g ) {
	pointTF2ToMsg( t.getOrigin(), g.position );
	quaternionTF2ToMsg( t.getRotation(), g.orientation );
}

class MarkerDetector {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher overlay_image_pub_;
		ros::Subscriber camera_info_sub_;
		ros::Publisher marker_pub_;

		cv::Ptr<cv::aruco::Dictionary> dictionary_;
		cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;

		dynamic_reconfigure::Server<ml_detector::SystemParamsConfig> dyncfg_system_settings_;
		dynamic_reconfigure::Server<ml_detector::DetectorParamsConfig> dyncfg_detector_settings_;

		image_transport::CameraSubscriber sub_camera_;

		int marker_seq_;
		int debug_seq_;

		int border_bits_;
		double marker_size_;
		double marker_spacing_;

		sensor_msgs::CameraInfo cam_info_;

		bool got_camera_info_;
		bool send_overlay_;
		bool send_detailed_tag_info_;
		bool show_rejected_;
		bool camera_rectified_;
		bool refine_strategy_;

		cv::Mat camera_matrix_;
		cv::Mat dist_coeffs_;

		std::vector< cv::Ptr< cv::aruco::Board > > board_list_;
		std::vector< std::vector< int > > board_configs_;

	public:
		MarkerDetector() :
			nh_(),
			nhp_("~"),
			it_(nhp_),
			got_camera_info_(false),
			border_bits_(0),
			marker_size_(0.0),
			marker_spacing_(0.0),
			dyncfg_detector_settings_(ros::NodeHandle(nhp_, "detector")),
			dyncfg_system_settings_(ros::NodeHandle(nhp_, "system")) {

			dyncfg_detector_settings_.setCallback(boost::bind(&MarkerDetector::callback_cfg_detector_settings, this, _1, _2));
			dyncfg_system_settings_.setCallback(boost::bind(&MarkerDetector::callback_cfg_system_settings, this, _1, _2));

			std::map< std::string, int > dictionary_ids = generate_dictionary_ids();
			std::string dictionary_id = "DICT_4X4_50";
			nhp_.param("board_config/dictionary", dictionary_id, dictionary_id);

			marker_seq_ = 0;
			debug_seq_ = 0;

			dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_ids[dictionary_id]));
			ROS_INFO("Dictionary: %s (size: %i)", dictionary_id.c_str(), dictionary_->bytesList.rows);

			if( readBoardConfig() && readBoardDefinitions() ) {
				ROS_INFO("Board definitions read sucessfully!");

				// Subscribe to input video feed and publish output video feed
				sub_camera_ = it_.subscribeCamera(ros::names::remap("image_raw"), 10, &MarkerDetector::detection_cb, this);

				overlay_image_pub_ = it_.advertise("image_overlay", 1);
				marker_pub_ = nhp_.advertise<ml_msgs::MarkerDetection> ("detected_markers", 100);

				ROS_INFO("Begining detection...");
			} else {
				ROS_ERROR("Error reading board configuration file.");
				ros::shutdown();
			}
		}

		~MarkerDetector() {
		}

		std::map< std::string, int > generate_dictionary_ids() {
			std::map< std::string, int > dict;
			dict["DICT_4X4_50"] = 0;
			dict["DICT_4X4_100"] = 1;
			dict["DICT_4X4_250"] = 2;
			dict["DICT_4X4_1000"] = 3;
			dict["DICT_5X5_50"] = 4;
			dict["DICT_5X5_100"] = 5;
			dict["DICT_5X5_250"] = 6;
			dict["DICT_5X5_1000"] = 7;
			dict["DICT_6X6_50"] = 8;
			dict["DICT_6X6_100"] = 9;
			dict["DICT_6X6_250"] = 10;
			dict["DICT_6X6_1000"] = 11;
			dict["DICT_7X7_50"] = 12;
			dict["DICT_7X7_100"] = 13;
			dict["DICT_7X7_250"] = 14;
			dict["DICT_7X7_1000"] = 15;
			dict["DICT_ARUCO_ORIGINAL"] = 16;

			return dict;
		}

		void callback_cfg_system_settings(ml_detector::SystemParamsConfig &config, uint32_t level) {
			send_overlay_ = config.publish_overlay;
			send_detailed_tag_info_ = config.send_detailed_tag_info;
			show_rejected_ = config.show_rejected;
			refine_strategy_ = config.refine_strategy;
			camera_rectified_ = config.camera_is_rectified;
		}

		void callback_cfg_detector_settings(ml_detector::DetectorParamsConfig &config, uint32_t level) {
			if(detectorParams_.empty())
				detectorParams_ = cv::aruco::DetectorParameters::create();

			detectorParams_->adaptiveThreshWinSizeMin = config.adaptive_thresh_win_size_min;
			detectorParams_->adaptiveThreshWinSizeMax = config.adaptive_thresh_win_size_max;
			detectorParams_->adaptiveThreshWinSizeStep = config.adaptive_thresh_win_size_step;
			detectorParams_->adaptiveThreshConstant = config.adaptive_thresh_constant;

			detectorParams_->minMarkerPerimeterRate = config.min_marker_perimeter_rate;
			detectorParams_->maxMarkerPerimeterRate = config.max_marker_perimeter_rate;

			detectorParams_->polygonalApproxAccuracyRate = config.polygonal_approx_accuracy_rate;
			detectorParams_->minCornerDistanceRate = config.min_corner_distance_rate;
			detectorParams_->minMarkerDistanceRate = config.min_marker_distance_rate;
			detectorParams_->minDistanceToBorder = config.min_distance_to_border;

			detectorParams_->markerBorderBits = config.marker_border_bits;
			detectorParams_->minOtsuStdDev = config.min_otsu_std_dev;
			detectorParams_->perspectiveRemovePixelPerCell = config.perspective_remove_pixel_per_cell;
			detectorParams_->perspectiveRemoveIgnoredMarginPerCell = config.perspective_remove_ignored_margin_per_cell;

			detectorParams_->maxErroneousBitsInBorderRate = config.max_erroneous_bits_in_border_rate;
			detectorParams_->errorCorrectionRate = config.error_correction_rate;

			ROS_WARN("Corner Refinement Method is currently unused");
			//detectorParams_->cornerRefinementMethod = config.corner_refinement_method;
			detectorParams_->cornerRefinementWinSize = config.corner_refinement_win_size;
			detectorParams_->cornerRefinementMaxIterations = config.corner_refinement_max_iterations;
			detectorParams_->cornerRefinementMinAccuracy = config.corner_refinement_min_accuracy;
		}

		//Pulled shamelessly from ar_sys (Sahloul)
		tf2::Transform getTF(const cv::Mat &Rvec, const cv::Mat &Tvec) {
			cv::Mat rot(3, 3, CV_64FC1);
			cv::Rodrigues(Rvec, rot);

			cv::Mat rotate_to_sys(3, 3, CV_64FC1);
			/*
			// Fixed the rotation to meet the ROS system
			// Doing a basic rotation around X with theta=PI
			// By Sahloul
			// See http://en.wikipedia.org/wiki/Rotation_matrix for details

			//	1	0	0
			//	0	-1	0
			//	0	0	-1
			rotate_to_sys.at<double>(0,0) = 1.0;
			rotate_to_sys.at<double>(0,1) = 0.0;
			rotate_to_sys.at<double>(0,2) = 0.0;
			rotate_to_sys.at<double>(1,0) = 0.0;
			rotate_to_sys.at<double>(1,1) = -1.0;
			rotate_to_sys.at<double>(1,2) = 0.0;
			rotate_to_sys.at<double>(2,0) = 0.0;
			rotate_to_sys.at<double>(2,1) = 0.0;
			rotate_to_sys.at<double>(2,2) = -1.0;
			rot = rot*rotate_to_sys.t();
			*/
			tf2::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
								  rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
								  rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

			tf2::Vector3 tf_orig(Tvec.at<double>(0,0), Tvec.at<double>(1,0), Tvec.at<double>(2,0));

			return tf2::Transform(tf_rot, tf_orig);
		}

		bool readBoardConfig(void) {
			bool success = false;

			nhp_.param("board_config/border_bits", border_bits_, border_bits_);
			nhp_.param("board_config/marker_size", marker_size_, marker_size_);
			nhp_.param("board_config/marker_spacing", marker_spacing_, marker_spacing_);

			ROS_INFO("Board configuration:");
			ROS_INFO("  Boarder bits: %i", border_bits_);
			ROS_INFO("  Marker size: %0.4f", marker_size_);
			ROS_INFO("  Marker spacing: %0.4f", marker_spacing_);

			if( (border_bits_ > 0) && (marker_size_ > 0) && (marker_spacing_ > 0) ) {
				success = true;
			} else {
				ROS_ERROR("Invalid board configuration parameters");
			}

			return success;
		}

		bool readBoardDefinitions(void) {
			int board_id_gen = 0;
			int marker_id_gen = 0;
			int i = 0;
			std::string board_name = "boards/board_";

			while(nhp_.hasParam( board_name + std::to_string(i) + "/id")) {
				ROS_INFO("Loading configuration for board %i...", i);
				board_configs_.push_back(std::vector< int >(3));

				nhp_.getParam( board_name + std::to_string(i) + "/rows", board_configs_.at(i).at(BC_ROWS) );
				nhp_.getParam( board_name + std::to_string(i) + "/cols", board_configs_.at(i).at(BC_COLS) );

				//ROS_ASSERT(((board_configs_.at(i).at(BC_ROWS) > 0) && (board_configs_.at(i).at(BC_COLS) > 0), "Rows and cols must be valid integers (>0)"
				ROS_INFO("  Setting board_%i size: [%i, %i]", i, board_configs_.at(i).at(BC_ROWS), board_configs_.at(i).at(BC_COLS));

				int temp_id = 0;
				nhp_.getParam( board_name + std::to_string(i) + "/id", temp_id );
				if(temp_id < 0) {
					board_configs_.at(i).at(BC_ID) = board_id_gen++;	//Use the next free id and increment
				} else {
					board_configs_.at(i).at(BC_ID) = temp_id;
					board_id_gen = temp_id + 1;	//Set the new free id
				}
				ROS_INFO("  Setting board_%i id: %i", i, board_configs_.at(i).at(BC_ID));

				cv::Ptr<cv::aruco::GridBoard> gridboard =
					cv::aruco::GridBoard::create(board_configs_.at(i).at(BC_ROWS), board_configs_.at(i).at(BC_COLS), marker_size_, marker_spacing_, dictionary_);

				std::vector< int > temp_marker_ids;
				nhp_.getParam( board_name + std::to_string(i) + "/marker_ids", temp_marker_ids );
				if(temp_marker_ids.size() < 1) {
					for(int j = 0; j < ( board_configs_.at(i).at(BC_ROWS) * board_configs_.at(i).at(BC_COLS) ); j++)
						temp_marker_ids.push_back(marker_id_gen++);	//insert the next generated marker id, then increment

					gridboard->ids = temp_marker_ids;
					ROS_INFO("  Generating board_%i %li marker_ids", i, gridboard->ids.size());
				} else {
					gridboard->ids = temp_marker_ids;

					ROS_INFO("  Setting board_%i %li marker_ids", i, gridboard->ids.size());

					for (unsigned int j = 0; j < temp_marker_ids.size(); j++)	//Search to see if there was a higher marker defined
						if (temp_marker_ids.at(j) > marker_id_gen)
							marker_id_gen = temp_marker_ids.at(j) + 1;	//If there was, the next id as the new free id
				}

				board_list_.push_back( gridboard.staticCast<cv::aruco::Board>() );	//Add the board to the list
				i++;
			}

			return (i > 0);	//Definitions loaded OK!
		}

		void decode_camera_info(const sensor_msgs::CameraInfoConstPtr& msg) {
			//If we have a new camera header
			if(cam_info_.header != msg->header) {
				cam_info_ = *msg;

				//XXX: Here we are relying on the definition that ROS and OpenCV are both expecting 1x5 vectors
				cv::Mat_<double>(cam_info_.D).reshape(0,1).copyTo(dist_coeffs_);	//Create a 3xN matrix with the raw data and copy the data to the right location

				cv::Mat_<double> m;
				if(camera_rectified_) {
					m.push_back(cam_info_.P[0]);
					m.push_back(cam_info_.P[1]);
					m.push_back(cam_info_.P[2]);
					m.push_back(cam_info_.P[4]);
					m.push_back(cam_info_.P[5]);
					m.push_back(cam_info_.P[6]);
					m.push_back(cam_info_.P[8]);
					m.push_back(cam_info_.P[9]);
					m.push_back(cam_info_.P[10]);
				} else {
					for(int i = 0; i < 9; i++)	//Copy the raw data into the matrix
						m.push_back( cam_info_.K[i] );
				}

				m.reshape(0,3).copyTo(camera_matrix_);	//Reshape to 3x3 and copy the data to the right location
			} //Else, don't bother re-extracting
		}

		void detection_cb(const sensor_msgs::ImageConstPtr& msg_img, const sensor_msgs::CameraInfoConstPtr& msg_info) {
			decode_camera_info(msg_info);

			cv_bridge::CvImagePtr cv_ptr;

			try	{
				cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
			}

			catch (cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}

			//ROS_INFO("Detecting...");

			std::vector< int > ids;
			std::vector< std::vector< cv::Point2f > > corners;
			std::vector< std::vector< cv::Point2f > > rejected;

			// detect markers and estimate pose
			cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, detectorParams_, rejected);

			if(ids.size() > 0) {	//If markers were found
				ml_msgs::MarkerDetection md_out;	//Detected markers message

				for(int i = 0; i < board_list_.size(); i++) {	//Iterate through the known boards for matches
					int markersOfBoardDetected = 0;
					std::vector< int > tags_found;
					std::vector< double > tags_found_confidence;
					cv::Vec3d rvec;
					cv::Vec3d tvec;

					//ROS_INFO("Detecting for board %i", i);

					//TODO: Maybe worth having a throttled message to keep track of average performance

					if(board_list_.at(i)->ids.size() == 1) {	//If the defined board is only 1 id
						//Search the id list for the index of the single ID board
						std::vector<int>::iterator it = std::find( ids.begin(), ids.end(), board_list_.at(i)->ids.at(0) );

						if( it != ids.end() ) {	//A match was found in the image for id of the board
							int ind = std::distance( ids.begin(), it);	//Get the index of the itterator

							std::vector<cv::Vec3d> rvecs;	//EstimatePoseSingleMarkers expects a vector of markers, but we only have 1, so just trick it
							std::vector<cv::Vec3d> tvecs;
							std::vector< std::vector< cv::Point2f > > corner;

							rvecs.push_back(rvec);
							tvecs.push_back(tvec);
							corner.push_back(corners.at(ind));	//Pull out the corners of the marker at spcific index

							cv::aruco::estimatePoseSingleMarkers(corner, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

							if(send_detailed_tag_info_) {
								tags_found.push_back( ids.at(ind) );
								tags_found_confidence.push_back( 1.0f );	//We don't have a way to get a good rating of tag confidence
							}

							markersOfBoardDetected = 1;
							rvec = rvecs.at(0);
							tvec = tvecs.at(0);
						}
					} else {	//Else it is a multi-marker board
						if(refine_strategy_)
						    cv::aruco::refineDetectedMarkers(cv_ptr->image, board_list_[i], corners, ids, rejected, camera_matrix_, dist_coeffs_);

						markersOfBoardDetected = cv::aruco::estimatePoseBoard(corners, ids, board_list_[i], camera_matrix_, dist_coeffs_, rvec, tvec);

						if(markersOfBoardDetected > 0) {
							//Allocate the adjustment vector (u)
							cv::Vec3d adj_vec;

							//Adjust the calculated position to move it to the center of the board
							adj_vec[0] = ( ( board_configs_.at(i).at(BC_ROWS) / 2.0 ) * marker_size_ ) + ( ( board_configs_.at(i).at(BC_ROWS) - 1 ) * marker_spacing_ / 2 );
							adj_vec[1] = ( ( board_configs_.at(i).at(BC_COLS) / 2.0 ) * marker_size_ ) + ( ( board_configs_.at(i).at(BC_COLS) - 1 ) * marker_spacing_ / 2 );
							adj_vec[2] = 0;

							//Rotate the adjustment to match the camera frame
							cv::Mat rot(3, 3, CV_64FC1);	//Allocate the rotation matrix (r)
							cv::Rodrigues(rvec, rot);	//Get the rotation matrix from the rodrigues vector
							cv::Matx31d rot_vec(cv::Matx33d(rot) * cv::Matx31d(adj_vec));	//Perform v = r * u

							//Apply the adjustment
							tvec += cv::Vec3d(rot_vec(0,0), rot_vec(1,0), rot_vec(2,0)); //Perform t = += v

							if(send_detailed_tag_info_) {	//Just to allow the user to get a bit more speed from the system
								tags_found = board_list_[i]->ids;
								tags_found_confidence.resize( tags_found.size(), 0.0f );

								for(int j = 0; j < tags_found.size(); j++) {
									std::vector<int>::iterator it = std::find( ids.begin(), ids.end(), tags_found.at(j) );

									if( it != ids.end() )
										tags_found_confidence.at(j) = 1.0f;	//We don't have a way to get a good rating of tag confidence
								}
							}
						}
					}

					if(markersOfBoardDetected > 0) {
						//Add the current marker to the detection message
						ml_msgs::Marker marker_out;

						//TODO: Should see if we can include tag data here
						marker_out.marker_id = board_configs_.at(i).at(BC_ID);	//The id of the board found
						marker_out.rows = board_configs_.at(i).at(BC_ROWS);	//The number of rows of tags of the board found
						marker_out.cols = board_configs_.at(i).at(BC_COLS);	//The number of cols of tags of the board found
						marker_out.marker_confidence = ( (double)markersOfBoardDetected ) / ( board_configs_.at(i).at(BC_ROWS) * board_configs_.at(i).at(BC_COLS) );	//Return the ratio of markers found for this board
						marker_out.tag_ids = tags_found;
						marker_out.tag_confidence = tags_found_confidence;
						poseTF2ToMsg( getTF( cv::Mat(rvec), cv::Mat(tvec) ), marker_out.pose );

						md_out.markers.push_back(marker_out);

						//==-- draw results
						if(send_overlay_ && (overlay_image_pub_.getNumSubscribers() > 0) ) {
							//ROS_INFO("Found board %i", i);
							cv::aruco::drawAxis(cv_ptr->image, camera_matrix_, dist_coeffs_, rvec, tvec, marker_size_ * 0.5f);
						}
					}
				}

				if(md_out.markers.size() > 0) {
					//Transmit the detection message
					md_out.header.stamp = msg_img->header.stamp;
					md_out.header.frame_id = msg_img->header.frame_id;
					md_out.header.seq = ++marker_seq_;

					marker_pub_.publish(md_out);
				}
			}

			//Only send the debug image once
			if(send_overlay_ && (overlay_image_pub_.getNumSubscribers() > 0) ) {
				if(ids.size() > 0) {
					cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
				}

				if(show_rejected_ && rejected.size() > 0)
					cv::aruco::drawDetectedMarkers(cv_ptr->image, rejected, cv::noArray(), cv::Scalar(100, 0, 255));

				//==-- Output modified video stream
				overlay_image_pub_.publish(cv_ptr->toImageMsg());
			}
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "ml_detector");
	MarkerDetector md;

	ros::spin();

	return 0;
}
