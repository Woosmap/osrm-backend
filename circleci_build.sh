#!/usr/bin/env bash

# build and push osrm lib on gemfury

if [ -z ${GEMFURY_TOKEN} ]; then
    echo "Missing or blank GEMFURY_TOKEN variable."
    exit -1
fi

echo current tag ${CIRCLE_TAG} tag
echo current tag ${CIRCLE_BRANCH} branch

if [ -z ${CIRCLE_TAG} ]; then
    echo "Tag is present."

fi

#if [ "${CIRCLE_BRANCH}" ==  "master" ]; then
#    eval "$(aws ecr get-login --no-include-email --region us-east-1)"
#    docker build -t ${ECR_REPOSITORY}:master .
#    docker push ${ECR_REPOSITORY}:master
#    curl -X POST http://stage-manager.woosmap.com:8080/hooks/deploy_stage
#elif [ "${CIRCLE_BRANCH}" ==  "develop" ]; then
#    eval "$(aws ecr get-login --no-include-email --region us-east-1)"
#    docker build -t ${ECR_REPOSITORY}:develop .
#    docker push ${ECR_REPOSITORY}:develop
#    curl -X POST http://stage-manager.woosmap.com:8080/hooks/deploy_develop
#fi

#VERSION=5.22.1
##BRANCH_TAG=v5.22.0-customsnapping.5
#PACKAGE_FILE_NAME=osrm-${VERSION}
#
##apt-get update && apt-get install -y git build-essential git cmake pkg-config \
##    libbz2-dev libstxxl-dev libstxxl1v5 libxml2-dev \
##    libzip-dev libboost-all-dev lua5.2 liblua5.2-dev libtbb-dev \
##    libluabind-dev libluabind0.9.1v5
#
##git clone https://github.com/Project-OSRM/osrm-backend.git
##cd osrm-backend
#
#mkdir -p build
#cd build
#cmake ..
#cpack -G DEB -P osrm -R ${VERSION} -D CPACK_PACKAGE_CONTACT=devproduit@woosmap.com -D CPACK_DEBIAN_PACKAGE_SHLIBDEPS=ON \
#-D CPACK_PACKAGE_FILE_NAME=${PACKAGE_FILE_NAME}
#
#curl -F package=@./${PACKAGE_FILE_NAME}.deb https://${GEMFURY_TOKEN}@push.fury.io/webgeoservices/
#
#cd ..
#rm -rf build/
#

