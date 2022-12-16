set -x
if [ $# -eq 2 ]; then 
CONFIGFILE=$1
BUS_ID=$2
else 
CONFIGFILE='all_sensor.json'
BUS_ID='1'
fi
sudo docker run --rm --name sensor_pi --device /dev/i2c-$BUS_ID --privileged -e TZ=Asia/Tokyo -it -v `pwd`/../catkin_ws:/catkin_ws -w /catkin_ws sensor_pi_pico bash -c "catkin_make; source /catkin_ws/devel/setup.bash; roslaunch sensor_pi sensor_pi.launch config_path:=\`rospack find sensor_pi\`/config/$CONFIGFILE bus_id:=$BUS_ID"
