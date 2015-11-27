FROM ubuntu:14.04
RUN apt-get update && apt-get install --fix-missing -y install xsltproc libxerces-c-dev xsdcxx libboost-filesystem1.57.0 libboost-system1.57.0 cmake build-essentials git
RUN git clone https://github.com/IGNF/lidarformat.git
RUN cd lidarformat && mkdir build && cd build && cmake .. && make install
RUN rm -rf lidarformat
RUN apt-get clean
