#!/usr/bin/env bash

set -e

cd build
cmake ..
cmake --build .
cmake --build . --target install
# Run cucumber tests with npm
npm install
npm link
npm run test