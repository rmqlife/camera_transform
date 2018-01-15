import rosbag
import os, re
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
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
        
    def save_image_topic(self, dirname, t):
        rgb_dir = "images"
        if not os.path.exists(rgb_dir):
            os.mkdir(rgb_dir)
        count=0
        tt_data = np.array([])
        bridge = CvBridge()
        for _, msg, t in self.obj.read_messages(topics=[t]):
            tt_data = np.vstack((tt_data,msg.data)) if tt_data.size else np.array(msg.data)
            cv_img = bridge.imgmsg_to_cv2(msg)
            cv_img = cv2.cvtColor(cv_img,cv2.COLOR_BGR2RGB)
            cv2.imwrite(os.path.join(rgb_dir + "/%04i.jpg" % count), cv_img)
            count = count + 1
            print(count)
        return
if __name__ == '__main__':
    bag = Bag('expert_flat.bag')
    #bag.save_topic(filename='robot_arms',t='/yumi/ikSloverVel_controller/ee_cart_position')
    bag.save_image_topic(dirname='robot_arms',t='/camera/image/rgb_611205001943')
