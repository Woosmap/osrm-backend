#!/usr/bin/env bash

set -e
docker compose -f docker-compose-test.yml build osrm-backend-test
docker compose -f docker-compose-test.yml run --rm osrm-backend-test

