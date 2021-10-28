# syntax=docker/dockerfile:1.3
FROM ubuntu:20.04 as builder

ARG DEBIAN_FRONTEND="noninteractive"
ARG VERSION="5.26.0-WGS1"
ENV TZ="Europe/Paris"
#ENV PACKAGE_FILE_NAME="osrm-wgs-${VERSION}"

RUN apt-get update &&  apt-get install -y curl build-essential git cmake pkg-config \
          libbz2-dev libstxxl-dev libstxxl1v5 libxml2-dev \
          libzip-dev libboost-all-dev lua5.2 liblua5.2-dev libtbb-dev \
          libluabind-dev libluabind0.9.1d1 npm

COPY . /usr/src/app
WORKDIR /usr/src/app

RUN mkdir -p /usr/src/app/build && \
    mkdir -p /usr/src/app/build-tmp

RUN --mount=type=cache,target=/usr/src/app/build  \
    cd build && pwd && ls &&\
    cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_ASSERTIONS=On -DBUILD_TOOLS=On -DENABLE_LTO=On && \
    make -j4 install && \
    #cpack -G DEB -P osrmwgs -R ${VERSION} -D CPACK_PACKAGE_CONTACT=devproduit@woosmap.com -D CPACK_DEBIAN_PACKAGE_SHLIBDEPS=ON -D CPACK_PACKAGE_FILE_NAME=${PACKAGE_FILE_NAME} && \
    cp -r * ../build-tmp && \
    cd ../profiles && cp -r * /opt

FROM ubuntu:20.04 as exporter

COPY --from=builder /usr/src/app/build-tmp /build

FROM ubuntu:20.04

COPY --from=builder /usr/local /usr/local
COPY --from=builder /opt /opt

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends libboost-program-options1.71.0 libboost-regex1.71.0 \
        libboost-date-time1.71.0 libboost-chrono1.71.0 libboost-filesystem1.71.0 \
        libboost-iostreams1.71.0 libboost-thread1.71.0 expat liblua5.2-0 libtbb2 &&\
    rm -rf /var/lib/apt/lists/*
