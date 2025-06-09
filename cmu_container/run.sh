#!/bin/bash

REPOSITORY_NAME="$(basename "$(dirname -- "$( readlink -f -- "$0"; )")")"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

export HOST_UID=$(id -u)

docker compose -f "$SCRIPT_DIR/docker-compose.yml" up -d cmu_container
docker run -it --net=host --ipc=host --rm cmu_exploration:v1