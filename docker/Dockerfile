FROM ros:noetic

RUN apt update && apt install -y i2c-tools python3-smbus \
	python-is-python3 git build-essential python3-dev python3-pip
WORKDIR /
RUN git clone https://github.com/johnbryanmoore/VL53L0X_rasp_python.git
WORKDIR VL53L0X_rasp_python
RUN make

