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
		image_transport::Publisher image_pub_;

		cv::Ptr<cv::aruco::Dictionary> dictionary;
		cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

		int dictionaryId;
		bool showRejected;
		bool estimatePose;
		double markerLength;

		bool send_debug;

		cv::Mat camMatrix;
		cv::Mat distCoeffs;

	public:
		MarkerDetector() : it_(nh_) {
			// Subscrive to input video feed and publish output video feed
			image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &MarkerDetector::imageCb, this);
			image_pub_ = it_.advertise("/marker_mapper/image_debug", 1);

			send_debug = true;

			dictionaryId = DICT_4X4_50;		//Which dictionary definition to use
			showRejected = false;	//Show debug rejections
			estimatePose = true;	//Estimate pose
			markerLength = 0.10;	//Marker length (m)

			dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
			ROS_INFO("Dictionary size: [%i]", dictionary->bytesList.rows);

			detectorParams = cv::aruco::DetectorParameters::create();
			detectorParams->doCornerRefinement = true;
			//TODO: Set a decent default params file
			//bool readParamsOK = readDetectorParameters(detectorParams);

			//if(!readParamsOK) {
			//	ROS_ERROR("Invalid detector parameters file");
			//	return;
			//}

			camMatrix = (cv::Mat_<double>(3,3) << 759.1760609558013, 0, 287.5327100302847, 0, 758.6599948698546, 256.1236460408814, 0, 0, 1);
			distCoeffs = (cv::Mat_<double>(1,5) << -0.00163408680065829, -0.03733632554264633, 0.005811228334756181, -0.005397615176127214, 0);

			/* TODO: Load dynamically
			if(estimatePose) {
				bool readCamParamsOK = readCameraParameters(camMatrix, distCoeffs);

				if(!readCamParamsOK) {
					cerr << "Invalid camera file" << endl;
					return;
				}
			}
			*/
		}

		~MarkerDetector() {
			//cv::destroyWindow(OPENCV_WINDOW);
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

		bool readDetectorParameters(cv::Ptr<cv::aruco::DetectorParameters> &params) {
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

			return true;
		}

		void imageCb(const sensor_msgs::ImageConstPtr& msg) {
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
