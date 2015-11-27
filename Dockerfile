FROM ubuntu:14.04
RUN apt-get update && apt-get install --fix-missing -y xsltproc libxerces-c-dev xsdcxx libboost-filesystem1.55.0 libboost-system1.55.0 cmake build-essential git
RUN git clone https://github.com/IGNF/lidarformat.git
RUN cd lidarformat && mkdir build && cd build && cmake .. && make install
RUN rm -rf lidarformat
RUN apt-get clean
