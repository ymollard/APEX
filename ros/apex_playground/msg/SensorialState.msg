geometry_msgs/PoseStamped hand
apex_playground/CircularState ball
sensor_msgs/Joy joystick_1
sensor_msgs/Joy joystick_2
apex_playground/CircularState ergo
std_msgs/Float32 sound
std_msgs/UInt8 color


# Hand: 3 valeurs (X, Y, Z) x 10 points temporels = 30D
# Joystick: 2 valeurs (quantité de Right, et de Forward, le Left est un Right négatif) x 10 points temporels = 20D
# Ergo: 2 valeurs (angle, élongation) * 10 = 20D
# Ball: 2 valeurs (angle, élongation) * 10 = 20D
# Color: 1 valeur (Hue) * 10 = 10D
# Sound: 2 valeurs (Frequence et intensité) * 10 = 20D
