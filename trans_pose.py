import rosbag
from geometry_msgs.msg import PoseStamped

bag_file = "/home/robin/Datasets/MVSEC/flying3.bag"  # 替换成你的ROS bag文件路径
output_file = "/home/robin/Datasets/MVSEC/MVSEC_to_TUM/poses.txt"  # 保存输出的txt文件路径

with open(output_file, "w") as file:
    bag = rosbag.Bag(bag_file)
    for topic, msg, t in bag.read_messages(topics=["/davis/left/pose"]):  # 替换成你的pose消息的topic
        timestamp = msg.header.stamp.to_sec()
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        line = "{:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f}\n".format(
            timestamp, tx, ty, tz, qx, qy, qz, qw)
        
        file.write(line)

    bag.close()

print("Poses saved to", output_file)
