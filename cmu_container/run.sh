#!/bin/bash
set -e


docker compose -f "/home/cam/Desktop/coding/docker/cmu_container/docker-compose.yml" run -it --rm cmu_container
# docker run -it --net=host --ipc=host --rm cmu_exploration:v1