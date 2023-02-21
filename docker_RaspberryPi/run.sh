if [ $# -eq 1 ]; then 
CONFIGFILE=$1
else 
CONFIGFILE='all_sensor.json'
fi
BUS_ID='1'
HOST_IP=`ip addr show dev docker0 |  grep -Eo 'inet [0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}' |  grep -Eo '[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}'`
MASTER_IP=${MASTER_IP:-"$HOST_IP"}
NET_SETTING="--net=host"
ROS_SETTINGS="--env ROS_IP=${HOST_IP} --env ROS_MASTER_URI=http://${MASTER_IP}:11311"
DEVICE_SETTINGS="--device /dev/gpiomem --device /dev/i2c-$BUS_ID --privileged"
SCRIPT_DIR=$(cd $(dirname $0); pwd)
set -x
sudo docker run --rm --name sensor_pi -it -e TZ=Asia/Tokyo $NET_SETTING $DEVICE_SETTINGS $ROS_SETTINGS -v $SCRIPT_DIR/../catkin_ws:/catkin_ws -w /catkin_ws sensor_pi bash -c "catkin_make; source /catkin_ws/devel/setup.bash; roslaunch sensor_pi sensor_pi.launch config_path:=\`rospack find sensor_pi\`/config/$CONFIGFILE bus_id:=$BUS_ID bus_type:=smbus"
