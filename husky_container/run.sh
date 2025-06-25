#!/bin/bash
set -e


docker compose -f "/home/cam/Desktop/coding/docker/husky_container/docker-compose.yml" run -it --rm husky_container
# docker run -it --net=host --ipc=host --rm cmu_exploration:v1