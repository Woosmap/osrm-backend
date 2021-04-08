#!/usr/bin/env bash
set -e
# push osrm lib on gemfury and osrm-backend-wgs to ECR

ECR_REPOSITORY=${1}
BRANCH=${GITHUB_REF#refs/heads/}
echo "Deploying version ${VERSION} if Master, we are ${BRANCH}"

if [ -z ${GEMFURY_PUSH_TOKEN} ]; then
    echo "Missing or blank GEMFURY_PUSH_TOKEN variable."
    exit -1
elif [ -z "${ECR_REPOSITORY}" ]; then
    echo "Missing ECR_REPOSITORY parameter."
    echo "usage: deploy.sh <ECR_REPOSITORY>"
    exit 255]
fi

if [ "${BRANCH}" ==  "master" ]; then

    PACKAGE_FILE_NAME=osrm-wgs-${VERSION}

    cd ./out/build

    curl -F package=@./${PACKAGE_FILE_NAME}.deb https://${GEMFURY_PUSH_TOKEN}@push.fury.io/${GEMFURY_USERNAME}/

    aws ecr-public get-login-password --region us-east-1 | docker login --username AWS --password-stdin ${ECR_REPOSITORY}
    docker tag osrm-backend-wgs "${ECR_REPOSITORY}:${BRANCH}"
    docker push "${ECR_REPOSITORY}:${BRANCH}"
else
    echo "Branch is not master."
fi
