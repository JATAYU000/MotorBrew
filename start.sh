cd ~/cognipilot/cranium/
colcon build
source install/setup.bash
#ros2 launch b3rb_bringup robot.launch.py shelf_count:=4 initial_angle:=000.0
ros2 launch b3rb_bringup robot.launch.py send_buffer_limit:=100000000 max_qos_depth:=200 use_compression:=true use_sim_time:=false shelf_count:=4 initial_angle:=000.0
