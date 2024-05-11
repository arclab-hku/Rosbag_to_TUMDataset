import cv2
import os

def convert_gray_to_color(image_path):
    # 读取图像
    img = cv2.imread(image_path)
    # 检查图像是灰度图像
    if len(img.shape) == 2:
        # 转换为彩色图像
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img

def process_images(input_folder, output_folder):
    # 创建输出文件夹，如果不存在则创建
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # 遍历输入文件夹中的所有文件
    for filename in os.listdir(input_folder):
        if filename.endswith(('.png', '.jpg', '.jpeg')):  # 假设我们处理的是.png、.jpg或.jpeg格式的图像
            input_file_path = os.path.join(input_folder, filename)
            output_file_path = os.path.join(output_folder, filename)
            # 转换图像并保存
            img = convert_gray_to_color(input_file_path)
            cv2.imwrite(output_file_path, img)

# 使用示例
input_folder = "/home/robin/Datasets/MVSEC/MVSEC_to_TUM/grey"  # 输入文件夹路径
output_folder = "/home/robin/Datasets/MVSEC/MVSEC_to_TUM/rgb"  # 输出文件夹路径
process_images(input_folder, output_folder)