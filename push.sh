#!/bin/bash
# Manually build and push the Droidal image to GHCR.
#
# CI (.github/workflows/docker-publish.yml) does this automatically on push to
# main; use this script only when you want to publish from your own machine.
#
# One-time login (uses a GitHub Personal Access Token with write:packages):
#   echo "$GHCR_PAT" | docker login ghcr.io -u prlancas --password-stdin
#
# Usage:
#   ./push.sh              # builds + pushes ghcr.io/prlancas/droidalros:latest
#   TAG=v0.2 ./push.sh     # custom tag
set -euo pipefail

IMAGE="${IMAGE:-ghcr.io/prlancas/droidalros}"
TAG="${TAG:-latest}"

cd "$(dirname "$0")"
echo "Building ${IMAGE}:${TAG} ..."
docker build -t "${IMAGE}:${TAG}" .
echo "Pushing ${IMAGE}:${TAG} ..."
docker push "${IMAGE}:${TAG}"
echo "Done: ${IMAGE}:${TAG}"
