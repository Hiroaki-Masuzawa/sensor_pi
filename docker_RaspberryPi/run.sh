set -x
if [ $# -eq 1 ]; then 
CONFIGFILE=$1
else 
CONFIGFILE='all_sensor.json'
fi
BUS_ID='1'
sudo docker run --rm --name sensor_pi --device /dev/gpiomem  --device /dev/i2c-1 --privileged -e TZ=Asia/Tokyo -it -v `pwd`/../catkin_ws:/catkin_ws -w /catkin_ws sensor_pi bash -c "catkin_make; source /catkin_ws/devel/setup.bash; roslaunch sensor_pi sensor_pi.launch config_path:=\`rospack find sensor_pi\`/config/$CONFIGFILE bus_id:=$BUS_ID bus_type:=smbus"
