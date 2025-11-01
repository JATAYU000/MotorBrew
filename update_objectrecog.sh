cd ~/MotorBrew

git checkout object-detect-setup
git pull


echo "moving new files"
rm -rf ~/cognipilot/cranium/src/NXP_AIM_INDIA_2025/resource
rm ~/cognipilot/cranium/src/NXP_AIM_INDIA_2025/b3rb_ros_aim_india/b3rb_ros_object_recog.py
rm ~/cognipilot/cranium/src/NXP_AIM_INDIA_2025/setup.py

echo "copying new files"
cp -r ~/MotorBrew/NXP_AIM_INDIA_2025/resource ~/cognipilot/cranium/src/NXP_AIM_INDIA_2025/
cp ~/MotorBrew/NXP_AIM_INDIA_2025/b3rb_ros_aim_india/b3rb_ros_object_recog.py ~/cognipilot/cranium/src/NXP_AIM_INDIA_2025/b3rb_ros_aim_india/
cp ~/MotorBrew/NXP_AIM_INDIA_2025/setup.py  ~/cognipilot/cranium/src/NXP_AIM_INDIA_2025/

echo  "done"
