#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"

exec "$@"