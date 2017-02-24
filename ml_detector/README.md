# ml_detector
This node will detect and output pose estimates for defined ARUCO markers/boards

# Programs:
- marker_create: Runs a small app that dumps the selected dictionary in the current directory
	rosrun marker_localization marker_create_node
- marker_detector: Handles marker detection and outputs the selected "boards" to TF with respect to the camera frame
	rosrun marker_localization marker_detector_node
