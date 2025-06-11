#!/bin/bash
set -e

# Show full build logs
export BUILDKIT_PROGRESS=plain

# Build the service
docker compose -f /home/cam/Desktop/coding/docker/reu_container/docker-compose.yml build 
#/path/to/your/docker-compose.yml