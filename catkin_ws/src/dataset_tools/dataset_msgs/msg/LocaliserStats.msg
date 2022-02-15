std_msgs/Header header

string source

dataset_msgs/Observation observation
dataset_msgs/Innovation innovation
float32 confidence

# Row-major representation of the 3x3 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, yaw)
float64[9] covariance
