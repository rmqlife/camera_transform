import cv2
import matplotlib.pyplot as plt
depth = cv2.imread("data/2018-01-19-00-29-15/depth/0001.png",cv2.IMREAD_UNCHANGED)
plt.imshow(depth)
plt.show()
print(depth.shape)
