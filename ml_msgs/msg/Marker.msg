# This expresses a marker in a 3D space
int32 marker_id           # id of the marker detected
float64 marker_confidence  # confidence that this marker is identified correctly

int32 rows                # number of rows of tags in this marker
int32 cols                # number of cols of tags in this marker

int32[] tag_ids           # possible ids of the detected marker (empty if no id could be identified)
float64[] tag_confidence  # confidence of the possible ids in 0 -> 1 (empty if no id could be identified)

geometry_msgs/Pose pose   # pose of the detected marker/fiducial
