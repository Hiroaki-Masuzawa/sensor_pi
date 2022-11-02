FROM ros:noetic

RUN apt update && apt install i2c-tools python3-smbus -y
RUN apt install -y python-is-python3 git
WORKDIR /
RUN git clone https://github.com/johnbryanmoore/VL53L0X_rasp_python.git
WORKDIR VL53L0X_rasp_python
RUN make

