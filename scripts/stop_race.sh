# Function to check the last command status and exit if it failed 
check_status() {
if [ $? -ne 0 ]; then
	echo "Error: $1 failed"
	exit 1
	fi
}
# Stop and remove the first container
docker stop race_scene_sdk_container
check_status "docker stop race_scene_sdk_container"
# docker rm race_scene_sdk_container
# check_status "docker rm race_scene_sdk_container"

# Stop and remove the second container
docker stop race_car_sdk_container
check_status "docker stop race_car_sdk_container"
# docker rm race_car_sdk_container
# check_status "docker rm race_car_sdk_container"

# Stop and remove the third container
docker stop race_drone_sdk_container
check_status "docker stop race_drone_sdk_container"
# docker rm race_drone_sdk_container
# check_status "docker rm race_drone_sdk_container"

# Stop and remove the fourth container
# 可以自行设置是否删除容器，正常来讲race_user_sdk_container 镜像可以重复使用, 其他三个镜像每次使用都需要新建进行初始化
# docker stop race_user_sdk_container
# check_status "docker stop race_user_sdk_container"
# docker rm race_user_sdk_container
# check_status "docker rm race_user_sdk_container"

echo "All containers have been stopped and removed successfully."