cd MotorBrew/
git checkout hardware
git pull

echo "pulled changes"

rm ~/cognipilot/cranium/src/NXP_AIM_INDIA_2025/b3rb_ros_aim_india/b3rb_ros_warehouse.py 
cp ./NXP_AIM_INDIA_2025/b3rb_ros_aim_india/b3rb_ros_warehouse.py  ~/cognipilot/cranium/src/NXP_AIM_INDIA_2025/b3rb_ros_aim_india/b3rb_ros_warehouse.py 

echo "copied"

