#!/usr/bin/env bash

# build and push osrm lib on gemfury

VERSION=5.22.1
PACKAGE_FILE_NAME=osrm-wgs-${VERSION}

if [ -z ${GEMFURY_TOKEN} ]; then
    echo "Missing or blank GEMFURY_TOKEN variable."
    exit -1
fi



if [ "${CIRCLE_BRANCH}" ==  "master" ]; then
    mkdir -p build
    cd build
    cmake ..
    cpack -G DEB -P osrm -R ${VERSION} -D CPACK_PACKAGE_CONTACT=devproduit@woosmap.com -D CPACK_DEBIAN_PACKAGE_SHLIBDEPS=ON \
    -D CPACK_PACKAGE_FILE_NAME=${PACKAGE_FILE_NAME}

    curl -F package=@./${PACKAGE_FILE_NAME}.deb https://${GEMFURY_TOKEN}@push.fury.io/webgeoservices/
fi
