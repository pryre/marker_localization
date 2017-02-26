#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ml_msgs/Marker.h>
#include <ml_msgs/MarkerDetection.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <vector>
#include <string>
#include <map>

//static const std::string OPENCV_WINDOW = "Image window";

typedef enum {
	BC_ID = 0,
	BC_ROWS,
	BC_COLS
} board_config_names;

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

class MarkerDetector {
	private:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher debug_image_pub_;
		ros::Subscriber camera_info_sub_;
		ros::Publisher marker_pub_;

		cv::Ptr<cv::aruco::Dictionary> dictionary;
		cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

		int marker_seq;
		int debug_seq;

		int border_bits;
		double marker_size;
		double marker_spacing;

		bool got_camera_info;
		bool send_debug;
		bool show_rejected;
		bool camera_rectified;
		bool refine_strategy;

		std::vector< double > cam_info_K;
		std::vector< double > cam_info_D;
		cv::Mat camera_matrix;
		cv::Mat dist_coeffs;

		std::vector< cv::Ptr< cv::aruco::Board > > board_list;
		std::vector< std::vector< int > > board_configs;

	public:
		MarkerDetector() : nh_(ros::this_node::getName()), it_(nh_), got_camera_info(false) {
			std::map< std::string, int > dictionary_ids = generate_dictionary_ids();
			std::string dictionary_id;
			loadParam(nh_, "board_config/dictionary", dictionary_id);

			std::string marker_topic = "markers";
			std::string debug_image_topic = "image_debug";
			std::string camera_info_topic = "/camera_info";
			std::string input_image_topic = "/image";

			loadParam(nh_, "debug_image_topic", debug_image_topic);
			loadParam(nh_, "marker_topic", marker_topic);
			loadParam(nh_, "camera_info_topic", camera_info_topic);
			loadParam(nh_, "input_image_topic", input_image_topic);
			loadParam(nh_, "camera_is_rectified", camera_rectified);

			marker_seq = 0;
			debug_seq = 0;

			// Subscrive to input video feed and publish output video feed
			debug_image_pub_ = it_.advertise(debug_image_topic, 1);

			loadParam(nh_, "send_debug", send_debug);
			loadParam(nh_, "show_rejected", show_rejected);
			loadParam(nh_, "refine_strategy", refine_strategy);

			dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_ids[dictionary_id]));
			ROS_INFO("Dictionary size: [%i]", dictionary->bytesList.rows);

			detectorParams = cv::aruco::DetectorParameters::create();
			readDetectorParameters(nh_, detectorParams);

			bool board_config_ok = readBoardConfig(nh_);
			bool board_definitions_ok = readBoardDefinitions(nh_);

			if(!board_config_ok || !board_definitions_ok) {
				ROS_ERROR("Error reading board configuration file.");
				ros::shutdown();
			} else {
				ROS_INFO("Board definitions read sucessfully!");
			}

			camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo> ( camera_info_topic, 1, &MarkerDetector::camera_info_cb, this );
			marker_pub_ = nh_.advertise<ml_msgs::MarkerDectection> (marker_topic, 100);

			ROS_INFO("Waiting for camera info...");

			while(!got_camera_info && ros::ok()) {
				ros::spinOnce();
				ros::Rate(20).sleep();
			}

			ROS_INFO("Recieved camera info!");

			ROS_INFO("Begining detection...");

			image_sub_ = it_.subscribe(input_image_topic, 1, &MarkerDetector::image_cb, this);
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

		//Pulled shamelessly from ar_sys (Sahloul)
		tf::Transform getTF(const cv::Mat &Rvec, const cv::Mat &Tvec) {
			cv::Mat rot(3, 3, CV_64FC1);
			cv::Rodrigues(Rvec, rot);

			cv::Mat rotate_to_sys(3, 3, CV_64FC1);

			// Fixed the rotation to meet the ROS system
			// Doing a basic rotation around X with theta=PI
			// By Sahloul
			// See http://en.wikipedia.org/wiki/Rotation_matrix for details

			//	1	0	0
			//	0	-1	0
			//	0	0	-1
			//rotate_to_sys.at<double>(0,0) = 1.0;
			//rotate_to_sys.at<double>(0,1) = 0.0;
			//rotate_to_sys.at<double>(0,2) = 0.0;
			//rotate_to_sys.at<double>(1,0) = 0.0;
			//rotate_to_sys.at<double>(1,1) = -1.0;
			//rotate_to_sys.at<double>(1,2) = 0.0;
			//rotate_to_sys.at<double>(2,0) = 0.0;
			//rotate_to_sys.at<double>(2,1) = 0.0;
			//rotate_to_sys.at<double>(2,2) = -1.0;
			//rot = rot*rotate_to_sys.t();

			tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
				rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
				rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

			tf::Vector3 tf_orig(Tvec.at<double>(0,0), Tvec.at<double>(1,0), Tvec.at<double>(2,0));

			return tf::Transform(tf_rot, tf_orig);
		}

		bool readBoardConfig(ros::NodeHandle &n) {
			loadParam(n, "board_config/border_bits", border_bits);
			loadParam(n, "board_config/marker_size", marker_size);
			loadParam(n, "board_config/marker_spacing", marker_spacing);

			return (border_bits > 0) && (marker_size > 0) && (marker_spacing > 0);
		}

		bool readBoardDefinitions(ros::NodeHandle &n) {
			int board_id_gen = 0;
			int marker_id_gen = 0;
			int i = 0;
			std::string board_name = "boards/board_";

			while(n.hasParam( board_name + std::to_string(i) + "/id")) {
				ROS_INFO("Loading configuration for board %i...", i);
				board_configs.push_back(std::vector< int >(3));

				n.getParam( board_name + std::to_string(i) + "/rows", board_configs.at(i).at(BC_ROWS) );
				n.getParam( board_name + std::to_string(i) + "/cols", board_configs.at(i).at(BC_COLS) );

				//ROS_ASSERT(((board_configs.at(i).at(BC_ROWS) > 0) && (board_configs.at(i).at(BC_COLS) > 0), "Rows and cols must be valid integers (>0)"
				ROS_INFO("  Setting board_%i size: [%i, %i]", i, board_configs.at(i).at(BC_ROWS), board_configs.at(i).at(BC_COLS));

				int temp_id = 0;
				n.getParam( board_name + std::to_string(i) + "/id", temp_id );
				if(temp_id < 0) {
					board_configs.at(i).at(BC_ID) = board_id_gen++;	//Use the next free id and increment
				} else {
					board_configs.at(i).at(BC_ID) = temp_id;
					board_id_gen = temp_id + 1;	//Set the new free id
				}
				ROS_INFO("  Setting board_%i id: %i", i, board_configs.at(i).at(BC_ID));

				cv::Ptr<cv::aruco::GridBoard> gridboard =
					cv::aruco::GridBoard::create(board_configs.at(i).at(BC_ROWS), board_configs.at(i).at(BC_COLS), marker_size, marker_spacing, dictionary);

				std::vector< int > temp_marker_ids;
				n.getParam( board_name + std::to_string(i) + "/marker_ids", temp_marker_ids );
				if(temp_marker_ids.size() < 1) {
					for(int j = 0; j < ( board_configs.at(i).at(BC_ROWS) * board_configs.at(i).at(BC_COLS) ); j++)
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

				board_list.push_back( gridboard.staticCast<cv::aruco::Board>() );	//Add the board to the list
				i++;
			}

			return (i > 0);	//Definitions loaded OK!
		}

		void readDetectorParameters(ros::NodeHandle &n, cv::Ptr<cv::aruco::DetectorParameters> &params) {
			loadParam(n, "system/adaptive_thresh_win_size_min", params->adaptiveThreshWinSizeMin);
			loadParam(n, "system/adaptive_thresh_win_size_max", params->adaptiveThreshWinSizeMax);
			loadParam(n, "system/adaptive_thresh_win_size_step", params->adaptiveThreshWinSizeStep);
			loadParam(n, "system/adaptive_thresh_constant", params->adaptiveThreshConstant);

			loadParam(n, "system/min_marker_perimeter_rate", params->minMarkerPerimeterRate);
			loadParam(n, "system/max_marker_perimeter_rate", params->maxMarkerPerimeterRate);

			loadParam(n, "system/polygonal_approx_accuracy_rate", params->polygonalApproxAccuracyRate);
			loadParam(n, "system/min_corner_distance_rate", params->minCornerDistanceRate);
			loadParam(n, "system/min_marker_distance_rate", params->minMarkerDistanceRate);
			loadParam(n, "system/min_distance_to_border", params->minDistanceToBorder);

			loadParam(n, "system/marker_border_bits", params->markerBorderBits);
			loadParam(n, "system/min_otsu_std_dev", params->minOtsuStdDev);
			loadParam(n, "system/perspective_remove_pixel_per_cell", params->perspectiveRemovePixelPerCell);
			loadParam(n, "system/perspective_remove_ignored_margin_per_cell", params->perspectiveRemoveIgnoredMarginPerCell);

			loadParam(n, "system/max_erroneous_bits_in_border_rate", params->maxErroneousBitsInBorderRate);
			loadParam(n, "system/error_correction_rate", params->errorCorrectionRate);

			loadParam(n, "system/do_corner_refinement", params->doCornerRefinement);
			loadParam(n, "system/corner_refinement_win_size", params->cornerRefinementWinSize);
			loadParam(n, "system/corner_refinement_max_iterations", params->cornerRefinementMaxIterations);
			loadParam(n, "system/corner_refinement_min_accuracy", params->cornerRefinementMinAccuracy);
		}

		void camera_info_cb(const sensor_msgs::CameraInfoConstPtr& msg) {
			//XXX: Here we are relying on the definition that ROS and OpenCV are both expecting 1x5 vectors
			cv::Mat_<double>(msg->D).reshape(0,1).copyTo(dist_coeffs);	//Create a 3xN matrix with the raw data and copy the data to the right location

			cv::Mat_<double> m;
			if(camera_rectified) {
				m.push_back(msg->P[0]);
				m.push_back(msg->P[1]);
				m.push_back(msg->P[2]);
				m.push_back(msg->P[4]);
				m.push_back(msg->P[5]);
				m.push_back(msg->P[6]);
				m.push_back(msg->P[8]);
				m.push_back(msg->P[9]);
				m.push_back(msg->P[10]);
			} else {
				for(int i = 0; i < 9; i++)	//Copy the raw data into the matrix
					m.push_back( msg->K[i] );
			}

			m.reshape(0,3).copyTo(camera_matrix);	//Reshape to 3x3 and copy the data to the right location

			got_camera_info = true;	//Allow processing to begin
		}

		void image_cb(const sensor_msgs::ImageConstPtr& msg) {
			cv_bridge::CvImagePtr cv_ptr;

			try	{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
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
			cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams, rejected);

			if(ids.size() > 0) {	//If markers were found
				ml_msgs::MarkerDectection md_out;	//Detected markers message

				for(int i = 0; i < board_list.size(); i++) {	//Iterate through the known boards for matches
					int markersOfBoardDetected = 0;
					cv::Vec3d rvec;
					cv::Vec3d tvec;

					//ROS_INFO("Detecting for board %i", i);

					//TODO: Maybe worth having a throttled message to keep track of average performance

					if(board_list.at(i)->ids.size() == 1) {	//If the defined board is only 1 id
						//Search the id list for the index of the single ID board
						std::vector<int>::iterator it = std::find( ids.begin(), ids.end(), board_list.at(i)->ids.at(0) );

						if( it != ids.end() ) {	//A match was found in the image for id of the board
							int ind = std::distance( ids.begin(), it);	//Get the index of the itterator

							std::vector<cv::Vec3d> rvecs;	//EstimatePoseSingleMarkers expects a vector of markers, but we only have 1, so just trick it
							std::vector<cv::Vec3d> tvecs;
							std::vector< std::vector< cv::Point2f > > corner;

							rvecs.push_back(rvec);
							tvecs.push_back(tvec);
							corner.push_back(corners.at(ind));	//Pull out the corners of the marker at spcific index

							cv::aruco::estimatePoseSingleMarkers(corner, marker_size, camera_matrix, dist_coeffs, rvecs, tvecs);

							markersOfBoardDetected = 1;
							rvec = rvecs.at(0);
							tvec = tvecs.at(0);
						}
					} else {	//Else it is a multi-marker board
						if(refine_strategy)
						    cv::aruco::refineDetectedMarkers(cv_ptr->image, board_list[i], corners, ids, rejected, camera_matrix, dist_coeffs);

						markersOfBoardDetected = cv::aruco::estimatePoseBoard(corners, ids, board_list[i], camera_matrix, dist_coeffs, rvec, tvec);

						if(markersOfBoardDetected > 0) {
							//Allocate the adjustment vector (u)
							cv::Vec3d adj_vec;

							//Adjust the calculated position to move it to the center of the board
							adj_vec[0] = ( ( board_configs.at(i).at(BC_ROWS) / 2 ) * marker_size ) + ( ( board_configs.at(i).at(BC_ROWS) - 1 ) * marker_spacing / 2 );
							adj_vec[1] = ( ( board_configs.at(i).at(BC_COLS) / 2 ) * marker_size ) + ( ( board_configs.at(i).at(BC_COLS) - 1 ) * marker_spacing / 2 );
							adj_vec[2] = 0;

							//Rotate the adjustment to match the camera frame
							cv::Mat rot(3, 3, CV_64FC1);	//Allocate the rotation matrix (r)
							cv::Rodrigues(rvec, rot);	//Get the rotation matrix from the rodrigues vector
							cv::Matx31d rot_vec(cv::Matx33d(rot) * cv::Matx31d(adj_vec));	//Perform v = r * u

							//Apply the adjustment
							tvec += cv::Vec3d(rot_vec(0,0), rot_vec(1,0), rot_vec(2,0)); //Perform t = += v
						}
					}

					if(markersOfBoardDetected > 0) {
						//Add the current marker to the detection message
						ml_msgs::Marker marker_out;

						//TODO: Should see if we can include tag data here
						marker_out.id = board_configs.at(i).at(BC_ID);	//The id of the board found
						marker_out.rows = board_configs.at(i).at(BC_ROWS);	//The number of rows of tags of the board found
						marker_out.cols = board_configs.at(i).at(BC_COLS);	//The number of cols of tags of the board found
						marker_out.marker.marker_confidence.push_back( ( (double)markersOfBoardDetected ) / ( board_configs.at(i).at(BC_ROWS) * board_configs.at(i).at(BC_COLS) ) );	//Return the ratio of markers found for this board
						poseTFToMsg( getTF( cv::Mat(rvec), cv::Mat(tvec) ), marker_out.pose );

						md_out.push_back(marker_out);

						//==-- draw results
						if(send_debug && (debug_image_pub_.getNumSubscribers() > 0) ) {
							//ROS_INFO("Found board %i", i);
							cv::aruco::drawAxis(cv_ptr->image, camera_matrix, dist_coeffs, rvec, tvec, marker_size * 0.5f);
						}
					}
				}

				//Transmit the detection message
				md_out.header.stamp = msg->header.stamp;
				md_out.header.frame_id = msg->header.frame_id;
				md_out.header.seq = ++marker_seq;

				marker_pub_.publish(md_out);
			}

			//Only send the debug image once
			if(send_debug && (debug_image_pub_.getNumSubscribers() > 0) ) {
				if(ids.size() > 0) {
					cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
				}

				if(show_rejected && rejected.size() > 0)
					cv::aruco::drawDetectedMarkers(cv_ptr->image, rejected, cv::noArray(), cv::Scalar(100, 0, 255));

				//==-- Output modified video stream
				debug_image_pub_.publish(cv_ptr->toImageMsg());
			}
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "ml_detector");
	MarkerDetector md;

	ros::spin();

	return 0;
}
