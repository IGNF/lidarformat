FROM ubuntu:14.04
RUN apt-get -y -qq update \
&&  apt-get --fix-missing -y install xsltproc libxerces-c-dev xsdcxx libboost-test1.55-dev libboost1.55-dev libboost-filesystem1.55-dev libboost-system1.55-dev cmake build-essential git \
&&  git clone https://github.com/IGNF/lidarformat.git \
&&  cd lidarformat \
&&  mkdir build \
&&  cd build \
&& cmake -DBUILD_TESTS=OFF  -DCMAKE_BUILD_TYPE=Release .. \
&& make install \
&& rm -rf lidarformat \
&& apt-get -y purge xsltproc libxerces-c-dev xsdcxx libboost-test1.55-dev libboost1.55-dev libboost-filesystem1.55-dev libboost-system1.55-dev cmake build-essential git \
&& apt-get clean \
&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*


