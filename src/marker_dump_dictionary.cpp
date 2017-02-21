#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>

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
    int markerId = 0;
    int borderBits = 1;
    int markerSize = 400;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(DICT_4X4_50));	//TODO: Dynamic!

	for(int i = 0; i < dictionary->bytesList.rows; i++) {
		std::stringstream out;
		out << "./marker_4x4_" << i << ".png";

		cv::Mat markerImg;
		cv::aruco::drawMarker(dictionary, i, markerSize, markerImg, borderBits);
		cv::imwrite(out.str(), markerImg);
	}
    return 0;
}
