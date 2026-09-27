#!/bin/bash
# Builds (if needed) and runs the ROS 2 Humble simulation container.
#
#   docker/run.sh            open a shell in the container, or attach if it is already running
#   docker/run.sh --build    rebuild the image first
#
# The workspace src folder is shared with the container at ~/lunabot_ws/src, so edits on
# either side are visible on both. The container's build/, install/, and log/ live in a
# docker volume and never touch the host's build.
set -e

IMAGE="lunabot:humble"
CONTAINER="lunabot_humble"
VOLUME="lunabot_humble_ws"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$(cd "${REPO_DIR}/.." && pwd)"

if ! command -v docker >/dev/null 2>&1; then
    echo "docker is not installed:  sudo apt install docker.io && sudo usermod -aG docker \$USER"
    exit 1
fi
if ! docker info >/dev/null 2>&1; then
    echo "cannot reach the docker daemon. Make sure you are in the docker group, then open a"
    echo "new terminal or run: newgrp docker   (do not run this script with sudo)"
    exit 1
fi

if [ "$1" = "--build" ] || ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "Building ${IMAGE} ..."
    docker build -f "${REPO_DIR}/docker/Dockerfile" -t "${IMAGE}" \
        --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" \
        --build-arg RENDER_GID="$(getent group render | cut -d: -f3)" "${REPO_DIR}"
fi

# Attach to a running container, unless it was started from an older image
if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
    if [ "$(docker inspect --format '{{.Image}}' "${CONTAINER}")" = "$(docker image inspect --format '{{.Id}}' "${IMAGE}")" ]; then
        exec docker exec -it "${CONTAINER}" bash
    fi
    echo "Container ${CONTAINER} is running on an older image, stopping it ..."
    docker stop "${CONTAINER}" >/dev/null
fi

# Let Gazebo, RViz, and the GUI reach the host display
command -v xhost >/dev/null 2>&1 && xhost +local: >/dev/null
docker volume create "${VOLUME}" >/dev/null

# Optional host resources: git over ssh, git identity, and the desktop's Ubuntu fonts
EXTRA=()
[ -S "${SSH_AUTH_SOCK:-}" ] && EXTRA+=(-v "${SSH_AUTH_SOCK}:/ssh-agent" -e SSH_AUTH_SOCK=/ssh-agent)
[ -d "${HOME}/.ssh" ] && EXTRA+=(-v "${HOME}/.ssh:/home/lunabot/.ssh:ro")
[ -f "${HOME}/.gitconfig" ] && EXTRA+=(-v "${HOME}/.gitconfig:/home/lunabot/.gitconfig:ro")
[ -d /usr/share/fonts/truetype/ubuntu ] && EXTRA+=(-v "/usr/share/fonts/truetype/ubuntu:/usr/share/fonts/truetype/ubuntu-host:ro")

exec docker run -it --rm \
    --name "${CONTAINER}" \
    --net=host \
    --ipc=host \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/dri \
    -v "${VOLUME}:/home/lunabot/lunabot_ws" \
    -v "${SRC_DIR}:/home/lunabot/lunabot_ws/src" \
    "${EXTRA[@]}" \
    "${IMAGE}" "$@"
