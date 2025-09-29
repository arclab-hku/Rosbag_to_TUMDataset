import cv2
import os

# 设置图片文件夹路径
input_folder = '/home/robin/r3live_dataset/rosbag_to_tum/degenerate_seq_02/depth'
output_folder = '/home/robin/r3live_dataset/rosbag_to_tum/degenerate_seq_02/half/depth'

# 如果输出文件夹不存在，创建一个
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# 遍历文件夹中的所有文件
for filename in os.listdir(input_folder):
    # 拼接完整文件路径
    file_path = os.path.join(input_folder, filename)
    
    # 读取图片
    img = cv2.imread(file_path)
    
    # 确保文件是图片格式
    if img is not None:
        # 调整图片大小为640x512
        resized_img = cv2.resize(img, (640, 512))
        
        # 保存调整后的图片到输出文件夹，保持文件名不变
        cv2.imwrite(os.path.join(output_folder, filename), resized_img)

print("所有图片调整完成！")
