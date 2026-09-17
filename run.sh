#!/bin/bash
# Bring up the whole Droidal stack (micro-ROS agent + SLAM container).
#
#   ./run.sh            # start in the foreground (Ctrl-C to stop)
#   ./run.sh -d         # start detached / in the background
#   ./run.sh down       # stop and remove the containers
#   ./run.sh shell      # Bash shell in the running droidal container
#   ./run.sh dev -d     # local source mounted into the stack
#   ./run.sh dev --build -d  # rebuild locally, then start with source mounted
#   ./run.sh dev down   # stop the local-development stack
#
# `dev` is for developing on this machine: changes under mnt/ are used after a
# droidal restart, without waiting for a published image.  Pass `--build` only
# after changing Dockerfile, requirements.txt, or other image-baked files.
set -euo pipefail
cd "$(dirname "$0")"

compose=(docker compose)
if [[ "${1:-}" == "dev" || "${1:-}" == "local" ]]; then
  shift
  compose+=( -f docker-compose.yml -f docker-compose.dev.yml )
fi

case "${1:-up}" in
  shell)
    # `docker compose exec` doesn't run the image entrypoint, so source ROS
    # explicitly and leave the user in an interactive shell.
    exec "${compose[@]}" exec droidal bash -lc 'source /opt/ros/jazzy/setup.bash && exec bash -i'
    ;;
  down|stop|restart|logs|ps|build|pull|config)
    exec "${compose[@]}" "$@"
    ;;
  *)
    exec "${compose[@]}" up "$@"
    ;;
esac
