#!/bin/bash
set -e

# Show full build logs
export BUILDKIT_PROGRESS=plain

# Build the service
docker compose -f /path/to/your/docker-compose.yml build 