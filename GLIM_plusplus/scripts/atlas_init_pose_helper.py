#!/usr/bin/env python3
# =============================================================================
# atlas_init_pose_helper.py — DEPRECATED
#
# This helper is no longer used by the launch file. The Hitch Sensor
# Dome fork now performs Atlas-Duo-driven initialization in C++:
#
#   - glim_ros2/src/glim_ros/glim_ros.cpp subscribes to ins_pose_topic
#     (default /pose) at startup and calls
#     OdometryEstimationIMU::set_init_state() with the first valid pose.
#   - glim/src/glim/odometry/initial_state_estimation.cpp's NaiveInit
#     waits for force_init=true and never derives gravity from the
#     accelerometer, so a moving-start bag works correctly.
#
# See GLIM_plusplus/docs/moving_start_initialization.md for the full design.
#
# This stub is kept only because the launch file in earlier revisions
# of the fork referenced it; removing the file would break a clean
# `git pull` for someone with stale local state. It exits with a
# non-zero code if invoked, with a pointer to the new mechanism.
# =============================================================================

import sys


_DEPRECATION = """\
[atlas-init] This script is DEPRECATED.

The Hitch Sensor Dome fork now performs Atlas-Duo-driven initialization
in C++ (glim_ros2/src/glim_ros/glim_ros.cpp), not via runtime config
patching. To use it:

  ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py
        ins_pose_topic:=/pose

(That's the default; you generally don't need to override it.)

For diagnostics, see scripts/check_init_stationarity.py.
For design rationale, see GLIM_plusplus/docs/moving_start_initialization.md.
"""


def main() -> int:
    sys.stderr.write(_DEPRECATION)
    return 2


if __name__ == "__main__":
    sys.exit(main())
