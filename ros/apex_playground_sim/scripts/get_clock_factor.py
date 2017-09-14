import rospy
import time
from rosgraph_msgs.msg import Clock
from collections import deque
from numpy import mean, array

rospy.init_node('clock_rate_measure')

win_size = 50
simulated_time = deque(maxlen=win_size)
real_time = deque(maxlen=win_size)

rate = 10.

def cb_clock(msg):
    global simulated_time, real_time
    simulated_time.append(msg.clock.to_sec())
    real_time.append(time.time())

rospy.Subscriber('/clock', Clock, cb_clock)

i = 1
rospy.loginfo("Measuring speed factor on {}-sized samples...".format(win_size))
while not rospy.is_shutdown():
   if len(simulated_time) == win_size and i == 0:
      delta_sim = simulated_time[-1] - simulated_time[0]
      delta_real = real_time[-1] - real_time[0]
      ratio = round(delta_sim / delta_real, 2)
      rospy.loginfo("Current speed: x{}".format(ratio))
   i = (i+1) % win_size
   time.sleep(1./rate)
