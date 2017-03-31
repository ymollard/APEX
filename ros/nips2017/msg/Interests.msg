# Name of interests
string[] names
std_msgs/UInt32 num_iterations

# For each space, for each iteration, value of the interest in [0., 1.]
# interests[iteration + space*iteration]
std_msgs/Float32[] interests  
