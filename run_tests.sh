#!/usr/bin/env bash

# Build the osrm lib
mkdir -p build
cd build
cmake ..
cmake --build .
cmake --build . --target install

# Run cucumber tests with npm
npm install
npm link
npm run test