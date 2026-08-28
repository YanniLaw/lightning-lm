#!/usr/bin/env bash

set -Eeuo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${repository_root}"

build_jobs="${LIGHTNING_BUILD_JOBS:-2}"
if [[ ! "${build_jobs}" =~ ^[1-9][0-9]*$ ]]; then
    echo "LIGHTNING_BUILD_JOBS must be a positive integer, got: ${build_jobs}" >&2
    exit 2
fi

load_limit="${LIGHTNING_BUILD_LOAD_LIMIT:-$((build_jobs * 2))}"
if [[ ! "${load_limit}" =~ ^[1-9][0-9]*$ ]]; then
    echo "LIGHTNING_BUILD_LOAD_LIMIT must be a positive integer, got: ${load_limit}" >&2
    exit 2
fi

minimum_free_disk_gb="${LIGHTNING_MIN_FREE_DISK_GB:-10}"
if [[ ! "${minimum_free_disk_gb}" =~ ^[0-9]+$ ]]; then
    echo "LIGHTNING_MIN_FREE_DISK_GB must be a non-negative integer, got: ${minimum_free_disk_gb}" >&2
    exit 2
fi

available_disk_kb="$(df -Pk . | awk 'NR == 2 {print $4}')"
minimum_free_disk_kb="$((minimum_free_disk_gb * 1024 * 1024))"
if ((available_disk_kb < minimum_free_disk_kb)); then
    echo "Not enough free disk space: at least ${minimum_free_disk_gb} GiB is required." >&2
    df -h . >&2
    exit 3
fi

ros_distro="${ROS_DISTRO:-jazzy}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
if [[ -f "${ros_setup}" ]]; then
    # shellcheck disable=SC1090
    # ROS 2 Jazzy setup scripts read variables that may be unset.
    set +u
    source "${ros_setup}"
    set -u
elif ! command -v colcon >/dev/null 2>&1; then
    echo "ROS setup was not found at ${ros_setup}, and colcon is not available in PATH." >&2
    exit 4
else
    echo "Warning: ${ros_setup} was not found; using the current shell environment." >&2
fi

for command_name in colcon nice ionice; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "Required command is missing: ${command_name}" >&2
        exit 4
    fi
done

export CMAKE_BUILD_PARALLEL_LEVEL="${build_jobs}"
export MAKEFLAGS="-j${build_jobs} -l${load_limit}"

echo "Low-impact local build"
echo "  compiler jobs: ${build_jobs}"
echo "  make load limit: ${load_limit}"
echo "  CPU priority: nice 10"
echo "  I/O priority: best-effort 7"
free -h
swapon --show || true
df -h .

exec nice -n 10 ionice -c 2 -n 7 \
    colcon build \
        --executor sequential \
        --parallel-workers 1 \
        --packages-select lightning \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=OFF \
            -DLIGHTNING_WITH_PANGOLIN=ON
