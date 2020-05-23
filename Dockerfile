FROM nvidia/opengl:base-ubuntu18.04

RUN apt -y update && apt -y install \
cmake gcc-7 g++-7 libboost-all-dev libeigen3-dev libgoogle-glog-dev libglew-dev libglfw3-dev \
libprotobuf-dev protobuf-compiler \
x11-utils libxcursor-dev libxrandr-dev libxinerama-dev libxi-dev \
libatlas-base-dev libsuitesparse-dev \
clang-format \
gcc-7-multilib g++-7-multilib

# Install Ceres from source
ADD ./3rdparty/ceres/ceres-solver/ source

RUN \
    cd source && \
    mkdir ceres-bin && \
    cd ceres-bin && \
    cmake ../ && \
    make -j10 && \
    make test && \
    make install

# Clean up APT when done.
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# make sure x11 is accessible https://nelkinda.com/blog/xeyes-in-docker/
RUN useradd -ms /bin/bash user
ENV DISPLAY :0
USER user

# TODO install ceres here. ref https://hub.docker.com/r/ewhitmire/ceres-solver/dockerfile/