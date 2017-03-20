# ml_detector
This node will detect and output pose estimates for defined ARUCO markers/boards

# Programs:
- marker_create: Runs a small app that dumps the selected dictionary in the current directory
	rosrun marker_localization marker_create_node
- marker_detector: Handles marker detection and outputs the selected "boards" to TF with respect to the camera frame
	rosrun marker_localization marker_detector_node

# Performance:
The marker_detector node will analyse the image frame for of the markers in the dictionary. If any tags are found, they are compared to the list of boards to identify any matches.
The performance is therefor dependant on 2 variables; the deictionary size (which will increase the time taken to find indivitual markers), and the number board definitions (as each definition is checked against the list of found tags).

# Notes:
Board estimates may currently not work due to upstream bug:
	Add useExtrinsicGuess flag in estimatePoseBoard() for aruco and charuco
	commit b89181dfad6fe4adfb4218426908eed2c0046c3a 