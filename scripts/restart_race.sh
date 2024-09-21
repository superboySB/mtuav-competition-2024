# Function to check the last command status and exit if it failed 
check_status() {
if [ $? -ne 0 ]; then
	echo "Error: $1 failed"
	exit 1
	fi
}
docker restart race_scene_sdk_container
docker restart race_car_sdk_container
docker restart race_drone_sdk_container
docker restart race_user_sdk_container

echo "All containers have been restarted successfully."