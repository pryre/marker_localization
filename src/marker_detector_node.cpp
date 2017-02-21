#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/aruco.hpp>

#include <vector>
#include <string>

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
		double markerLength;

		bool got_camera_info;
		bool send_debug;

		std::vector< double > cam_info_K;
		std::vector< double > cam_info_D;
		cv::Mat camMatrix;
		cv::Mat distCoeffs;

	public:
		MarkerDetector() : nh_(ros::this_node::getName()), it_(nh_) {

			// Subscrive to input video feed and publish output video feed
			image_pub_ = it_.advertise("image_debug", 1);	//TODO: param

			send_debug = true;

			//TODO: Load as params
			dictionaryId = DICT_4X4_50;		//Which dictionary definition to use
			showRejected = false;	//Show debug rejections
			estimatePose = true;	//Estimate pose
			markerLength = 0.10;	//Marker length (m)

			dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
			ROS_INFO("Dictionary size: [%i]", dictionary->bytesList.rows);

			detectorParams = cv::aruco::DetectorParameters::create();
			readDetectorParameters(nh_, detectorParams);
			detectorParams->doCornerRefinement = true;

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
			cv::Mat_<double>(msg->D).reshape(0,1).copyTo(distCoeffs);	//Create a 3xN matrix with the raw data and copy the data to the right location

			cv::Mat_<double> m;
			for(int i = 0; i < 9; i++)	//Copy the raw data into the matrix
				m.push_back( msg->K[i] );
			m.reshape(0,3).copyTo(camMatrix);	//Reshape to 3x3 and copy the data to the right location

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
			std::vector< cv::Vec3d > rvecs;
			std::vector< cv::Vec3d > tvecs;

			// detect markers and estimate pose
			cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams, rejected);

			//TODO:
			//Right here the system should split the ids found into grouped definitions
			// based on the boards, etc. that the user has defined, then evaluate each
			// id set individually, giving a TF output for each
			//All id sets should be output with TFs relative to the camera frame

			if(ids.size() > 0) {
				//TODO: Maybe worth having a throttled message to keep track of average performance
				if(estimatePose && (ids.size() > 0)) {
					cv::aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
					//TODO: TF output
				}
			}

			//==-- draw results
			if(send_debug) {	//TODO: & has subscribers
				if(ids.size() > 0) {
					cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

					if(estimatePose) {
						for(unsigned int i = 0; i < ids.size(); i++)
							cv::aruco::drawAxis(cv_ptr->image, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
					}
				}

				if(showRejected && rejected.size() > 0)
					cv::aruco::drawDetectedMarkers(cv_ptr->image, rejected, cv::noArray(), cv::Scalar(100, 0, 255));

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
