#!/bin/bash

export HOST_UID=$(id -u)
export BUILDKIT_PROGRESS=plain

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
  
docker compose -f $SCRIPT_DIR/docker-compose.yml build 