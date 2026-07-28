# glim_ros2

Please open issues in the main GLIM repository: https://github.com/koide3/glim

Camera intake is excluded from this fork's default build. The default
`BUILD_WITH_CV_BRIDGE=OFF` does not find or link `cv_bridge` or
`image_transport`, and image subscription/deserialization code is not compiled.
Opting in requires a core `glim` built with `BUILD_WITH_OPENCV=ON`, the camera
packages installed manually, and `BUILD_WITH_CV_BRIDGE=ON`.

[![ROS2](https://github.com/koide3/glim_ros2/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/glim_ros2/actions/workflows/build.yml)
