FROM ubuntu:20.04 as builder

ARG DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Paris"

RUN apt-get update &&  apt-get install -y curl build-essential git cmake pkg-config \
          libbz2-dev libstxxl-dev libstxxl1v5 libxml2-dev \
          libzip-dev libboost-all-dev lua5.2 liblua5.2-dev libtbb-dev \
          libluabind-dev libluabind0.9.1d1

COPY . /usr/src/app
WORKDIR /usr/src/app

RUN mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j2 install

FROM ubuntu:20.04

COPY --from=builder /usr/local /usr/local
COPY --from=builder /opt /opt