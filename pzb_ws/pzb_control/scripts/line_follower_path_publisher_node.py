# #!/usr/bin/env python3

# import os
# import csv
# import math 
# import numpy as np

# from ament_index_python.packages import get_package_share_directory

# import rclpy
# from rclpy.node import Node

# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped, Vector3, Point
# from std_msgs.msg import Int32

# def get_R(ang):
#     return np.array([
#         [math.cos(ang), -math.sin(ang), 0],
#         [math.sin(ang), math.cos(ang), 0],
#         [0, 0, 1]])

# class LineFollowerPathPublisherNode(Node):

#     def __init__(self):
#         super().__init__('line_follower_path_publisher_node')

#         parent_frame = 'world'

#         self.lf_error_sub = self.create_subscription(
#             Int32, '/pzb/lf_err', self.lf_callback, 1
#         )

#         self.pose_sub = self.create_subscription(
#             Vector3, '/pzb_pose', self.pose_callback, 1
#         )

#         self.pose = Vector3(x = 0., y = 0., z = 0.)
#         self.lf_err = 0.

#         self.path_pub_ = self.create_publisher(Path, "/pzb/path_to_follow", 10)
#         self.timer = self.create_timer(1, self.timer_callback)

#         # self.waypoints_file_ = os.path.join(
#         #     get_package_share_directory('usv_control'),
#         #     'config',
#         #     'example.csv'
#         # )

#         self.path_ = Path()
#         self.path_.header.frame_id = parent_frame
#         self.path_.header.stamp = self.get_clock().now().to_msg()
#         for i in range(2):
#             pose_stmpd = PoseStamped()
#             pose_stmpd.header.frame_id = parent_frame
#             pose_stmpd.pose.position.x = 0.
#             pose_stmpd.pose.position.y = 0.
#             self.path_.poses.append(pose_stmpd)

#     def timer_callback(self):
#         goal = np.array([1, self.lf_err, 0])
#         goal_r = self.R.dot(goal)

#         self.path_.poses[0].pose.position = Point(x = self.pose.x, y = self.pose.y, z = 0.)
#         self.path_.poses[1].pose.position = Point(x = goal_r[0], y = goal_r[1], z = 0.)
        
#         self.path_pub_.publish(self.path_)

#     def lf_callback(self, msg):
#         self.lf_err = -msg.data / 25.
    
#     def pose_callback(self, msg):
#         self.pose = msg
#         self.R = get_R(msg.z)


# def main(args=None):
#     rclpy.init(args=args)

#     line_follower_path_publisher_node = LineFollowerPathPublisherNode()
#     rclpy.spin(line_follower_path_publisher_node)
#     line_follower_path_publisher_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()