#!/bin/bash
set -e


docker compose -f "/path/to/your/docker-compose.yml" run -it --rm liosam_cmu_container
# docker run -it --net=host --ipc=host --rm cmu_exploration:v1