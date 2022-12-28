FROM ros:noetic

RUN apt update && apt install -y i2c-tools python3-smbus \
	python-is-python3 git build-essential python3-dev python3-pip libusb-1.0*
RUN pip3 install smbus2 pyftdi