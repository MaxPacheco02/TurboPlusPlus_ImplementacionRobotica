### Puzzlebot Workspace

ssh puzzlebot@10.42.0.197

# para correr seguidor de linea
# pzb
ros2 launch pzb_vision color_detection_launch.py
ros2 run LineFollower LineDetection
./run_micro_agent.sh

# compu
ros2 launch pzb_control pzb_launch.py
ros2 run pzb_control line_follower_path_publisher_node
