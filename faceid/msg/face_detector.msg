#face_detector.msg

# adding a standard header that contains a time stamp (ROS time),
# a frame ID(string) and a sequence ID
Header header

# Coordinates of the evaluated person with covarince
geometry_msgs/PoseWithCovarianceStamped[] pos

# Depth of the evaluated person
float32[] depth

# The angle where the person is located in the image
float32[] angle

# The probability score of the recognized face
float64[] face_score

# The probability score of the recognized face
float64[] eye_score
