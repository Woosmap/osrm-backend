#!/usr/bin/env bash
set -e
# push osrm lib on gemfury and osrm-backend-wgs to ECR

ECR_REPOSITORY=${1}
VERSION=${2}
echo "Deploying version ${VERSION} if Master, we are ${BRANCH}"

if [ -z "${ECR_REPOSITORY}" ]; then
    echo "Missing ECR_REPOSITORY parameter."
    echo "usage: deploy.sh <ECR_REPOSITORY> <VERSION>"
    exit 255]
fi

aws ecr-public get-login-password --region us-east-1 | docker login --username AWS --password-stdin ${ECR_REPOSITORY}
docker tag osrm-backend-wgs "${ECR_REPOSITORY}:${VERSION}"
docker push "${ECR_REPOSITORY}:${VERSION}"

