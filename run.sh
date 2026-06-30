#!/bin/bash
# Bring up the whole Droidal stack (micro-ROS agent + SLAM container).
#
#   ./run.sh            # start in the foreground (Ctrl-C to stop)
#   ./run.sh -d         # start detached / in the background
#   ./run.sh down       # stop and remove the containers
#
# First run pulls/builds the image; after that it just starts.
set -euo pipefail
cd "$(dirname "$0")"

case "${1:-up}" in
  down|stop|logs|ps|build|pull)
    exec docker compose "$@"
    ;;
  *)
    exec docker compose up "$@"
    ;;
esac
