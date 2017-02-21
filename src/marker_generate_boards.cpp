#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>

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

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "marker_generate_boards");
	ros::NodeHandle nh(ros::this_node::getName());

	std::string output_directory;
	int num_boards = 0;

    int border_bits = 0;
	double marker_size = 0;
	double marker_spacing = 0;

	int output_height = 0;
	int output_width = 0;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(DICT_4X4_50));	//TODO: params

	if( !nh.getParam( "output_directory", output_directory ) ) {
		ROS_ERROR( "\"output_directory\" not set" );
		return 1;
	} else {
		ROS_INFO( "Generating boards is %s", output_directory.c_str() );
	}

	if( !nh.getParam( "boards/num_boards", num_boards ) ) {
		ROS_ERROR( "\"boards/num_boards\" not set" );
		return 1;
	} else {
		ROS_INFO( "Generating %i boards", num_boards );
	}

	if( !nh.getParam( "boards/border_bits", border_bits ) ) {
		ROS_ERROR( "\"boards/border_bits\" not set" );
		return 1;
	} else {
		ROS_INFO( "Generating using %i border spacing", border_bits );
	}

	if( !nh.getParam( "boards/marker_size", marker_size ) ) {
		ROS_ERROR( "\"boards/marker_size\" not set" );
		return 1;
	} else {
		ROS_INFO( "Generating markers at size %fm", marker_size );
	}

	if( !nh.getParam( "boards/marker_spacing", marker_spacing ) ) {
		ROS_ERROR( "\"boards/marker_spacing\" not set" );
		return 1;
	} else {
		ROS_INFO( "Generating markers with spacing %fm", marker_spacing );
	}

	if( !nh.getParam( "boards/output_height", output_height ) ) {
		ROS_ERROR( "\"boards/output_height\" not set" );
		return 1;
	} else {
		ROS_INFO( "Output image will be %ipx in height", output_height );
	}

	if( !nh.getParam( "boards/output_width", output_width ) ) {
		ROS_ERROR( "\"boards/output_width\" not set" );
		return 1;
	} else {
		ROS_INFO( "Output image will be %ipx in width", output_width );
	}

	for(int i = 0; i < num_boards; i++) {
		ROS_INFO( "Generating board_%i", i );

		int rows = 0;
		int cols = 0;
		std::vector<int> ids;
		std::stringstream board_name;
		board_name << "board_" << i;

		if( !nh.getParam( "boards/" + board_name.str() + "/rows" , rows ) ) {
			ROS_ERROR( "\"rows\" not set for board_%i", i );
			return 1;
		}

		if( !nh.getParam( "boards/" + board_name.str() + "/cols" , cols ) ) {
			ROS_ERROR( "\"cols\" not set for board_%i", i );
			return 1;
		}

		if( !nh.getParam( "boards/" + board_name.str() + "/ids" , ids ) ) {
			ROS_ERROR( "\"ids\" not set for board_%i", i );
			return 1;
		}

		//If the rows and cols are valid, and either the size matches number of ids (or generate ids)
		if( ( ( ( rows * cols ) == ids.size() ) || ( ids.size() == 0 ) ) && ( rows > 0 ) && ( cols > 0 ) ) {
			/*
			if( ( rows * cols ) == 1 ) {	//The only 1 marker is needed for this "board"
				int id = 0;

				if( ids.size() > 0 )
					id = ids.at(0);

				std::stringstream out;
				out << "./marker_4x4_" << i << ".png";

				cv::Mat markerImg;
				cv::aruco::drawMarker(dictionary, i, output_width, markerImg, borderBits);
				cv::imwrite(out.str(), markerImg);
			} else { }
			*/

			 cv::Ptr<cv::aruco::GridBoard> board =
				cv::aruco::GridBoard::create(rows, cols, marker_size, marker_spacing, dictionary);

			if(ids.size() > 0)
				board->ids = ids;

			cv::Mat boardImage;
			board->draw( cv::Size(output_width, output_height), boardImage, 10, border_bits );

			std::stringstream out;
			out << output_directory << board_name.str() << ".png";
			cv::imwrite(out.str(), boardImage);

		} else {
			ROS_ERROR( "\"ids\" length does not match specified size for board_%i", i );
			return 1;
		}
	}

    return 0;
}


