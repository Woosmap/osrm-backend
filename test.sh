#!/usr/bin/env bash

set -e

cd build

echo "BOOST tests ..."
cmake ..
make tests
cd unit_tests
echo "Starting contractor-tests"
./contractor-tests
echo "Starting customizer-tests"
./customizer-tests
echo "Starting engine-tests"
./engine-tests
echo "Starting extractor-tests"
./extractor-tests
echo "Starting library-contract-tests"
./library-contract-tests
echo "Starting library-customize-tests"
./library-customize-tests
echo "Starting library-extract-tests"
./library-extract-tests
echo "Starting library-partition-tests"
./library-partition-tests
echo "Starting library-tests"
./library-tests
echo "Starting partitioner-tests"
./partitioner-tests
echo "Starting server-tests"
./server-tests
echo "Starting storage-tests"
./storage-tests
echo "Starting updater-tests"
./updater-tests
echo "Starting util-tests"
./util-tests
cd ..

echo "Cucumber tests ..."
cmake ..
cmake --build .
cmake --build . --target install
# Run cucumber tests with npm
npm install
npm link
npm run test