#!/usr/bin/env bash

# build and push osrm lib on gemfury

if [ -z ${GEMFURY_PUSH_TOKEN} ]; then
    echo "Missing or blank GEMFURY_PUSH_TOKEN variable."
    exit -1
fi

BRANCH=${GITHUB_REF#refs/heads/}
if [ "${BRANCH}" ==  "master" ]; then
    VERSION=5.23.0
    PACKAGE_FILE_NAME=osrm-wgs-${VERSION}

    mkdir -p build
    cd build
    cmake ..
    cpack -G DEB -P osrmwgs -R ${VERSION} -D CPACK_PACKAGE_CONTACT=devproduit@woosmap.com -D CPACK_DEBIAN_PACKAGE_SHLIBDEPS=ON \
    -D CPACK_PACKAGE_FILE_NAME=${PACKAGE_FILE_NAME}

    curl -F package=@./${PACKAGE_FILE_NAME}.deb https://${GEMFURY_PUSH_TOKEN}@push.fury.io/${GEMFURY_USERNAME}/
else
    echo "Branch is not master."
fi
