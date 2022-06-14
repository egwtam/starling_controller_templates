#!/usr/bin/env bash
set -e

WORKDIR="$(cd "$(dirname "$0")" && pwd)/.."
OUTDIR="${WORKDIR}/../"

echo "Force CookieCutter to copy current templates out"
cookiecutter "$WORKDIR" --directory starling_template --no-input -f -o "$OUTDIR"
cookiecutter "$WORKDIR" --directory python_ros2_node_offboard_template --no-input -f -o "${OUTDIR}/starling_controller/starling_controller"
cookiecutter "$WORKDIR" --directory cpp_ros2_node_onboard_template --no-input -f -o "${OUTDIR}/starling_controller/starling_controller"
cookiecutter "$WORKDIR" --directory ros2_msgs_template --no-input -f -o "${OUTDIR}/starling_controller/starling_controller"


echo "Building CookieCutter Directory"
make -C "$OUTDIR/starling_controller" build