FROM ros:noetic

RUN apt update && apt install -y i2c-tools python3-smbus \
	python-is-python3 git build-essential python3-dev python3-pip libusb-1.0*
RUN pip3 install smbus2 pyftdi 
RUN apt install -y vim
WORKDIR /tmp
RUN git clone https://github.com/nonNoise/PyMCP2221A.git
WORKDIR /tmp/PyMCP2221A/pypi/main
RUN cp ../../PyMCP2221A/*.py PyMCP2221A/. && \
	sed -i.bk -e "596i \ \ \ \ \ \ \ \ buf[3 + 1] |= 0x01" PyMCP2221A/PyMCP2221A.py && \
	python3 setup.py build && \
	python3 setup.py install