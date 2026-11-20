#!/usr/bin/env bash
# Run from a disposable extraction on a Linux Pi. All fixture ROS traffic stays on a private loopback.
set -eo pipefail
stage=$(cd "$(dirname "$0")/../.." && pwd -P)
if [[ $stage != /home/pi/fc-operator-qualification.* || ! -f $stage/native/src/joy_utils/package.xml ]]; then
  echo "Use a separate fc-operator-qualification extraction with native/src/joy_utils." >&2
  exit 2
fi
prefix="$stage/native/install/joy_utils"
mounts=()
if [[ ${1:-} == --gateway-prefix ]]; then
  prefix=$(realpath -e "$2")
  [[ $prefix =~ ^/home/pi/pupper-fc-distributions/fc-operator-gateway-[a-f0-9]+/joy_utils$ ]]
  mounts=(--ro-bind "$prefix" "$prefix")
  shift 2
fi
if (( $# == 0 )); then
  echo "Choose build, test, or an explicit fixture command." >&2
  exit 2
fi
exec nice -n 18 ionice -c 3 bwrap --unshare-all --die-with-parent --clearenv \
  --ro-bind / / --tmpfs /home --bind "$stage" "$stage" "${mounts[@]}" --tmpfs /tmp --tmpfs /run --tmpfs /sys \
  --dev /dev --proc /proc --chdir "$stage" \
  --setenv HOME /tmp --setenv PATH /usr/local/bin:/usr/bin:/bin --setenv LANG C.UTF-8 \
  --setenv PUPPER_FC_FIXTURE_GATEWAY_PREFIX "$prefix" --setenv PUPPER_FC_ISOLATED_FIXTURE 1 --setenv ROS_DOMAIN_ID 112 --setenv ROS_LOCALHOST_ONLY 1 --setenv ROS_LOG_DIR /tmp/ros-logs \
  --setenv CMAKE_BUILD_PARALLEL_LEVEL 1 --setenv MAKEFLAGS -j1 \
  -- /bin/bash -ec '
    source /opt/ros/jazzy/setup.bash
    case "$1" in
      build)
        cd native
        exec colcon build --base-paths src --packages-select joy_utils --executor sequential \
          --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
        ;;
      test)
        source native/install/local_setup.bash
        exec ctest --test-dir native/build/joy_utils --output-on-failure -R "^(fc_|stop_|start_motion_|estop_parameters)"
        ;;
      *)
        prefix=$PUPPER_FC_FIXTURE_GATEWAY_PREFIX
        if [[ -d $prefix/lib ]]; then
          export AMENT_PREFIX_PATH="$prefix:${AMENT_PREFIX_PATH:-}" LD_LIBRARY_PATH="$prefix/lib:${LD_LIBRARY_PATH:-}"
          for site in "$prefix"/{lib,local/lib}/python*/{site,dist}-packages; do
            if [[ -d $site/joy_utils ]]; then export PYTHONPATH="$site:${PYTHONPATH:-}"; fi
          done
        fi
        exec "$@"
        ;;
    esac
  ' fc-operator-fixture "$@"
