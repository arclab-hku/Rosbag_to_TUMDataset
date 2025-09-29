#!/usr/bin/env python

import os
import cv2
import rosbag
import numpy as np
from sensor_msgs.msg import CompressedImage
import rospy

# 配置输出文件夹路径
output_dir = '/home/robin/r3live_dataset/rosbag_to_tum/degenerate_seq_02'
rgb_folder = os.path.join(output_dir, 'rgb')
os.makedirs(rgb_folder, exist_ok=True)
rgb_txt_path = os.path.join(output_dir, 'rgb.txt')

# 打开 ROS bag 文件
bag = rosbag.Bag('/home/robin/r3live_dataset/rosbag_to_tum/degenerate_seq_02/degenerate_seq_02.bag', 'r')

# 打开 rgb.txt 文件写入时间戳和图像文件名
with open(rgb_txt_path, 'w') as f:
    f.write("# timestamp filename\n")

    # 设定最大时间范围
    max_duration = 30  # 提取前 102 秒的图像

    # 获取 bag 开始时间
    start_time = None

    # 设定图像提取的跳过步数
    skip_step = 0  # 每提取 1 张跳过 4 张
    msg_count = 0  # 记录消息的计数

    # 遍历 bag 文件中的图像消息
    for topic, msg, t in bag.read_messages(topics=['/camera/image_color/compressed']):
        if start_time is None:
            start_time = t.to_sec()  # 记录第一个消息的时间戳

        # 计算当前消息时间相对于起始时间的时长
        current_time = t.to_sec() - start_time

        # 如果已经超过 102 秒，则停止提取
        if current_time > max_duration:
            break

        # 仅处理每 skip_step 条消息中的一条
        if skip_step!=0:
            if msg_count % skip_step == 0:
                # 处理压缩图像消息
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 解压缩图像

                # 获取时间戳并格式化为秒的浮点数，保留 4 位小数
                timestamp = round(msg.header.stamp.to_sec(), 4)

                # 创建图像文件名，保留 4 位小数
                image_filename = "{:.4f}.png".format(timestamp)
                image_path = os.path.join(rgb_folder, image_filename)

                # 保存图像到 rgb 文件夹
                cv2.imwrite(image_path, cv_image)

                # 将时间戳和图像文件名写入 rgb.txt
                f.write("{:.4f} rgb/{}\n".format(timestamp, image_filename))

                # 输出进度
                print(f"提取图像: {image_filename}")
        else:
            # 处理压缩图像消息
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 解压缩图像

            # 获取时间戳并格式化为秒的浮点数，保留 4 位小数
            timestamp = round(msg.header.stamp.to_sec(), 4)

            # 创建图像文件名，保留 4 位小数
            image_filename = "{:.4f}.png".format(timestamp)
            image_path = os.path.join(rgb_folder, image_filename)

            # 保存图像到 rgb 文件夹
            cv2.imwrite(image_path, cv_image)

            # 将时间戳和图像文件名写入 rgb.txt
            f.write("{:.4f} rgb/{}\n".format(timestamp, image_filename))

            # 输出进度
            print(f"提取图像: {image_filename}")

        msg_count += 1  # 递增消息计数器

print("前 102 秒的图像已成功提取并保存。")

# 关闭 bag 文件
bag.close()

