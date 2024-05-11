import os
import cv2
import numpy as np
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
 
# 定义文件路径
script_dir = os.getcwd()
rgb_path = os.path.join("/home/robin/Datasets/MVSEC/MVSEC_to_TUM", 'rgb')
depth_path = os.path.join("/home/robin/Datasets/MVSEC/MVSEC_to_TUM", 'depth')
 
# 创建 CvBridge 实例
bridge = CvBridge()
 
# 定义常量
DEPTH_ENCODING = '32FC1'
DEPTH_NORMALIZE_MIN = 0.0
DEPTH_NORMALIZE_MAX = 1.0
DEPTH_SCALE = 65535
 
num = 1
rgb_width = None
rgb_height = None
 
# 确保文件夹存在
if not os.path.exists(rgb_path):
    os.makedirs(rgb_path)
if not os.path.exists(depth_path):
    os.makedirs(depth_path)
 
with rosbag.Bag(os.path.join("/home/robin/Datasets/MVSEC", 'flying3.bag'), 'r') as bag:  #bag路径
    with open(os.path.join("/home/robin/Datasets/MVSEC/MVSEC_to_TUM", 'depth.txt'), 'w') as depth_time_file, open(os.path.join("/home/robin/Datasets/MVSEC/MVSEC_to_TUM", 'rgb.txt'), 'w') as rgb_time_file:
        for topic, msg, t in bag.read_messages():
            if topic == "/davis/left/depth_image_rect": #depth topic
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding=DEPTH_ENCODING)
                # 将深度图像范围归一化到0-1之间，并将NaN值替换为0
                cv_image_normalized = cv_image.copy()  # 创建归一化后的图像副本
                cv_image_normalized[np.isnan(cv_image_normalized)] = 0  # 将NaN值替换为0
                cv_image_normalized = cv2.normalize(cv_image_normalized, None, DEPTH_NORMALIZE_MIN, DEPTH_NORMALIZE_MAX, cv2.NORM_MINMAX)
                # 将深度图像范围归一化到0-1之间
                #cv_image_normalized = cv2.normalize(cv_image, None, DEPTH_NORMALIZE_MIN, DEPTH_NORMALIZE_MAX, cv2.NORM_MINMAX)
 
                # 获取深度图像尺寸
                depth_height, depth_width = cv_image_normalized.shape
 
                if rgb_width is not None and rgb_height is not None:
                    # 调整深度图像尺寸为与彩色图像相同
                    cv_image_normalized = cv2.resize(cv_image_normalized, (rgb_width, rgb_height), interpolation=cv2.INTER_LINEAR)
 
                # 将深度图像范围扩展到0-65535并转换为16位无符号整数（CV_16U）
                cv_image_mono16 = (cv_image_normalized * DEPTH_SCALE).astype(np.uint16)
 
                # 保存深度图像
                image_name = str(num) + '.png'
                cv2.imwrite(os.path.join(depth_path, image_name), cv_image_mono16)
 
                # 写入深度图像的时间戳
                timestr = "%.8f" % msg.header.stamp.to_sec()
                depth_time_file.write(timestr + " depth/" + image_name + "\n")
 
            if topic == "/davis/left/image_raw":  #rgb topic
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                rgb_height, rgb_width, _ = cv_image.shape
 
                # 进行深度图像几何校正和对齐操作，使其与彩色图像对齐
 
                # 保存彩色图像
                image_name = str(num) + '.png'
                cv2.imwrite(os.path.join(rgb_path, image_name), cv_image)
 
                # 写入彩色图像的时间戳
                timestr = "%.8f" % msg.header.stamp.to_sec()
                rgb_time_file.write(timestr + " rgb/" + image_name + "\n")
 
            num += 1
