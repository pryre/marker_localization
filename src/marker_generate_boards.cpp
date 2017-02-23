#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>

#include <vector>
#include <string>
#include <map>

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

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "marker_generate_boards");
	ros::NodeHandle nh(ros::this_node::getName());

	std::string output_directory;

	std::map< std::string, int > dictionary_ids = generate_dictionary_ids();
	std::string dictionary_id;


    int border_bits = 0;
	double marker_size = 0;
	double marker_spacing = 0;

	int output_height = 0;
	int output_width = 0;

	if( !nh.getParam( "board_config/dictionary", dictionary_id ) ) {
		ROS_ERROR( "\"board_config/dictionary\" not set" );
		return 1;
	} else {
		ROS_INFO( "Using dictionary %s", dictionary_id.c_str() );
	}

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_ids[dictionary_id]));	//TODO: params
	ROS_INFO("Dictionary size: [%i]", dictionary->bytesList.rows);

	if( !nh.getParam( "output_directory", output_directory ) ) {
		ROS_ERROR( "\"output_directory\" not set" );
		return 1;
	} else {
		ROS_INFO( "Generating boards is %s", output_directory.c_str() );
	}

	if( !nh.getParam( "board_config/border_bits", border_bits ) ) {
		ROS_ERROR( "\"board_config/border_bits\" not set" );
		return 1;
	} else {
		ROS_INFO( "Generating using %i border spacing", border_bits );
	}

	if( !nh.getParam( "board_config/marker_size", marker_size ) ) {
		ROS_ERROR( "\"board_config/marker_size\" not set" );
		return 1;
	} else {
		ROS_INFO( "Generating markers at size %fm", marker_size );
	}

	if( !nh.getParam( "board_config/marker_spacing", marker_spacing ) ) {
		ROS_ERROR( "\"board_config/marker_spacing\" not set" );
		return 1;
	} else {
		ROS_INFO( "Generating markers with spacing %fm", marker_spacing );
	}

	if( !nh.getParam( "board_config/output_height", output_height ) ) {
		ROS_ERROR( "\"board_config/output_height\" not set" );
		return 1;
	} else {
		ROS_INFO( "Output image will be %ipx in height", output_height );
	}

	if( !nh.getParam( "board_config/output_width", output_width ) ) {
		ROS_ERROR( "\"board_config/output_width\" not set" );
		return 1;
	} else {
		ROS_INFO( "Output image will be %ipx in width", output_width );
	}

	int board_id_gen = 0;
	int marker_id_gen = 0;
	int i = 0;
	std::string board_name = "board_";

	while(nh.hasParam( "boards/" + board_name + std::to_string(i) + "/id")) {
		int rows = 0;
		int cols = 0;
		int board_id = 0;

		ROS_INFO("Loading configuration for board %i...", i);

		nh.getParam( "boards/" + board_name + std::to_string(i) + "/rows", rows );
		nh.getParam( "boards/" + board_name + std::to_string(i) + "/cols", cols );

		ROS_INFO("  Setting board_%i size: [%i, %i]", i, rows, cols);

		int temp_id = 0;
		nh.getParam( "boards/" + board_name + std::to_string(i) + "/id", temp_id );
		if(temp_id < 0) {
			board_id = board_id_gen++;	//Use the next free id and increment
		} else {
			board_id = temp_id;
			board_id_gen = temp_id + 1;	//Set the new free id
		}
		ROS_INFO("  Setting board_%i id: %i", i, board_id);

		cv::Ptr<cv::aruco::GridBoard> gridboard =
			cv::aruco::GridBoard::create(rows, cols, marker_size, marker_spacing, dictionary);

		std::vector< int > temp_marker_ids;
		nh.getParam( "boards/" + board_name + std::to_string(i) + "/marker_ids", temp_marker_ids );
		if(temp_marker_ids.size() < 1) {
			for(int j = 0; j < ( rows * cols ); j++)
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

		cv::Mat boardImage;
		gridboard->draw( cv::Size(output_width, output_height), boardImage, 10, border_bits );

		cv::imwrite(output_directory + board_name + std::to_string(i) + ".png", boardImage);

		i++;
	}

    return 0;
}


