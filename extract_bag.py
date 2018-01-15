import rosbag
import os, re
from sensor_msgs.msg import Image
import numpy as np
import sys
# conda install libgcc
class Bag:
    def __init__(self,filename):
        self.obj = rosbag.Bag(filename)
        self.topics = self.obj.get_type_and_topic_info()[1].keys()
        print(self.topics)
    def save_topic(self,filename, t):
        tt_data = np.array([])
        print(t)
        for _, msg, t in self.obj.read_messages(topics=[t]):
            print(msg.data)
            tt_data = np.vstack((tt_data,msg.data)) if tt_data.size else np.array(msg.data)
        print("saved data", tt_data.shape)
        np.save(filename, tt_data)
        return
if __name__ == '__main__':
    bag = Bag('expert_flat.bag')
    bag.save_topic(filename='robot_arms',t='/yumi/ikSloverVel_controller/ee_cart_position')
