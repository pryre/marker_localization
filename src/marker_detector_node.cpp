#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/aruco.hpp>

#include <vector>
#include <string>
#include <sstream>

typedef enum {
	DICT_4X4_50 = 0,
	DICT_4X4_100,
	DICT_4X4_250,
	DICT_4X4_1000,
	DICT_5X5_50,
	DICT_5X5_100,
	DICT_5X5_250,
	DICT_5X5_1000,
	DICT_6X6_50,
	DICT_6X6_100,
	DICT_6X6_250,
	DICT_6X6_1000,
	DICT_7X7_50,
	DICT_7X7_100,
	DICT_7X7_250,
	DICT_7X7_1000,
	DICT_ARUCO_ORIGINAL
} aruco_dictionaries;

//static const std::string OPENCV_WINDOW = "Image window";

class MarkerDetector {
	private:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		ros::Subscriber camera_info_sub_;
		image_transport::Publisher image_pub_;

		cv::Ptr<cv::aruco::Dictionary> dictionary;
		cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

		int dictionaryId;

		bool showRejected;
		bool estimatePose;

		int border_bits;
		double marker_size;
		double marker_spacing;

		bool got_camera_info;
		bool send_debug;

		std::vector< double > cam_info_K;
		std::vector< double > cam_info_D;
		cv::Mat camera_matrix;
		cv::Mat dist_coeffs;

		std::vector< cv::Ptr< cv::aruco::Board > > board_list;

	public:
		MarkerDetector() : nh_(ros::this_node::getName()), it_(nh_) {

			// Subscrive to input video feed and publish output video feed
			image_pub_ = it_.advertise("image_debug", 1);	//TODO: param

			//TODO: Load as params
			send_debug = true;
			showRejected = false;	//Show debug rejections
			estimatePose = true;	//Estimate pose

			dictionaryId = DICT_4X4_50;		//Which dictionary definition to use
			dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
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

			camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo> ( "/usb_cam/camera_info", 1, &MarkerDetector::camera_info_cb, this );	//TODO: param

			ROS_INFO("Waiting for camera info...");
			got_camera_info = false;	//A check to see if we have received distortion and camera data

			while(!got_camera_info && ros::ok()) {
				ros::spinOnce();
				ros::Rate(20).sleep();
			}

			ROS_INFO("Recieved camera info!");

			ROS_INFO("Begining detection...");

			image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &MarkerDetector::image_cb, this);	//TODO: param
		}

		~MarkerDetector() {
		}

		/*
		tf::Transform ar_sys::getTf(const cv::Mat &Rvec, const cv::Mat &Tvec) {
			cv::Mat rot(3, 3, CV_32FC1);
			cv::Rodrigues(Rvec, rot);

			cv::Mat rotate_to_sys(3, 3, CV_32FC1);

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

			tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
				rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
				rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

			tf::Vector3 tf_orig(Tvec.at<double>(0,0), Tvec.at<double>(1,0), Tvec.at<double>(2,0));

			return tf::Transform(tf_rot, tf_orig);
		}
		*/

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

		bool readBoardConfig(ros::NodeHandle &n) {
			loadParam(n, "boards/border_bits", border_bits);
			loadParam(n, "boards/marker_size", marker_size);
			loadParam(n, "boards/marker_spacing", marker_spacing);

			return (border_bits > 0) && (marker_size > 0) && (marker_spacing > 0);
		}

		bool readBoardDefinitions(ros::NodeHandle &n) {
			int num_boards;
				bool check = true;

			loadParam(n, "boards/num_boards", num_boards);

			for(int i = 0; i < num_boards; i++) {
				int rows = 0;
				int cols = 0;
				std::vector<int> ids;

				std::stringstream board_name;
				board_name << "boards/board_" << i;

				check &= n.getParam( board_name.str() + "/rows" , rows );
				check &= n.getParam( board_name.str() + "/cols" , cols );
				check &= n.getParam( board_name.str() + "/ids" , ids );

				//If the rows and cols are valid, and either the size matches number of ids (or generate ids)
				if( ( ( ( rows * cols ) == ids.size() ) || ( ids.size() == 0 ) ) && ( rows > 0 ) && ( cols > 0 ) ) {
					if(check) {	//All parameters loaded correctly
						cv::Ptr<cv::aruco::GridBoard> gridboard =
							cv::aruco::GridBoard::create(rows, cols, marker_size, marker_spacing, dictionary);

						if(ids.size() > 0)
							gridboard->ids = ids;

						board_list.push_back( gridboard.staticCast<cv::aruco::Board>() );
					} else {
						ROS_ERROR("Definition of board %i has an error!", i);
						check = false;
					}
				} else {
					ROS_ERROR("List of ids is not empty, and does not match rows * cols, or they are is invalid");
					check = false;
				}
			}

			return check;	//Definitions loaded OK!
		}

		void readDetectorParameters(ros::NodeHandle &n, cv::Ptr<cv::aruco::DetectorParameters> &params) {
			/*
			params->adaptiveThreshWinSizeMin = 3;
			params->adaptiveThreshWinSizeMax = 23;
			params->adaptiveThreshWinSizeStep = 10;
			params->adaptiveThreshConstant = 7;
			params->minMarkerPerimeterRate = 0.03;
			params->maxMarkerPerimeterRate = 4.0;
			params->polygonalApproxAccuracyRate = 0.05;
			params->minCornerDistanceRate = 10.0;
			params->minDistanceToBorder = 3;
			params->minMarkerDistanceRate = 0.05;
			params->doCornerRefinement = true;
			params->cornerRefinementWinSize = 11;
			params->cornerRefinementMaxIterations = 30;
			params->cornerRefinementMinAccuracy = 0.1;
			params->markerBorderBits = 1;
			params->perspectiveRemovePixelPerCell = 8;
			params->perspectiveRemoveIgnoredMarginPerCell = 0.13;
			params->maxErroneousBitsInBorderRate = 0.04;
			params->minOtsuStdDev = 5.0;
			params->errorCorrectionRate = 0.6;
			*/

			loadParam(n, "system/adaptive_thresh_win_size_min", params->adaptiveThreshWinSizeMin);
			loadParam(n, "system/adaptive_thresh_win_size_max", params->adaptiveThreshWinSizeMax);
			loadParam(n, "system/adaptive_thresh_win_size_step", params->adaptiveThreshWinSizeStep);
			loadParam(n, "system/adaptive_thresh_constant", params->adaptiveThreshConstant);
			loadParam(n, "system/min_marker_perimeter_rate", params->minMarkerPerimeterRate);
			loadParam(n, "system/max_marker_perimeter_rate", params->maxMarkerPerimeterRate);
			loadParam(n, "system/polygonal_approx_accuracy_rate", params->polygonalApproxAccuracyRate);
			loadParam(n, "system/min_corner_distance_rate", params->minCornerDistanceRate);
			loadParam(n, "system/min_distance_to_border", params->minDistanceToBorder);
			loadParam(n, "system/min_marker_distance_rate", params->minMarkerDistanceRate);
			loadParam(n, "system/do_corner_refinement", params->doCornerRefinement);
			loadParam(n, "system/corner_refinement_win_size", params->cornerRefinementWinSize);
			loadParam(n, "system/corner_refinement_max_iterations", params->cornerRefinementMaxIterations);
			loadParam(n, "system/corner_refinement_min_accuracy", params->cornerRefinementMinAccuracy);
			loadParam(n, "system/marker_border_bits", params->markerBorderBits);
			loadParam(n, "system/perspective_remove_pixel_per_cell", params->perspectiveRemovePixelPerCell);
			loadParam(n, "system/perspective_remove_ignored_margin_per_cell", params->perspectiveRemoveIgnoredMarginPerCell);
			loadParam(n, "system/max_erroneous_bits_in_border_rate", params->maxErroneousBitsInBorderRate);
			loadParam(n, "system/min_otsu_std_dev", params->minOtsuStdDev);
			loadParam(n, "system/error_correction_rate", params->errorCorrectionRate);
		}

		void camera_info_cb(const sensor_msgs::CameraInfoConstPtr& msg) {
			//XXX: Here we are relying on the definition that ROS and OpenCV are both expecting 1x5 vectors
			cv::Mat_<double>(msg->D).reshape(0,1).copyTo(dist_coeffs);	//Create a 3xN matrix with the raw data and copy the data to the right location

			cv::Mat_<double> m;
			for(int i = 0; i < 9; i++)	//Copy the raw data into the matrix
				m.push_back( msg->K[i] );
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

			//TODO: //This does not work for a single marker board, need to trim id results and estimate using the normal marker function

			for(int i = 0; i < board_list.size(); i++) {
				int markersOfBoardDetected = 0;
				cv::Vec3d rvec;
				cv::Vec3d tvec;

				//TODO: Consider doing marker refinement
				//if(refindStrategy)
				//    aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix, distCoeffs);

				if(ids.size() > 0) {
					//TODO: Maybe worth having a throttled message to keep track of average performance
					//if(estimatePose && (ids.size() > 0)) {
					//	cv::aruco::estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs, rvecs, tvecs);
					//}

					markersOfBoardDetected =
						cv::aruco::estimatePoseBoard(corners, ids, board_list.at(i), camera_matrix, dist_coeffs, rvec, tvec);
				}

				//==-- draw results
				if(send_debug) {	//TODO: & has subscribers
					if(ids.size() > 0) {
						cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

						//if(estimatePose) {
						//	for(unsigned int i = 0; i < ids.size(); i++)
						//		cv::aruco::drawAxis(cv_ptr->image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_size * 0.5f);
						//}

						if(markersOfBoardDetected > 0)
							cv::aruco::drawAxis(cv_ptr->image, camera_matrix, dist_coeffs, rvec, tvec, marker_size * 2.0f);
					}

					if(showRejected && rejected.size() > 0)
						cv::aruco::drawDetectedMarkers(cv_ptr->image, rejected, cv::noArray(), cv::Scalar(100, 0, 255));
				}
			}

			//Only send the debug image once
			if(send_debug) {	//TODO: & has subscribers
				//==-- Output modified video stream
				image_pub_.publish(cv_ptr->toImageMsg());
			}
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "marker_mapper");
	MarkerDetector ic;

	ros::spin();

	return 0;
}
