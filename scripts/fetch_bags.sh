#!/bin/bash
# Copies recorded bags from the robot to this machine for review. Already copied bags are skipped
# and an interrupted copy resumes where it stopped.
#
#   scripts/fetch_bags.sh [robot_host]     default codetc@192.168.0.250

host=${1:-codetc@192.168.0.250}
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bag_root="$(cd "${script_dir}/../.." && pwd)/bags"

mkdir -p "${bag_root}"
rsync -av --partial --progress "${host}:lunabot_ws/src/bags/" "${bag_root}/"
