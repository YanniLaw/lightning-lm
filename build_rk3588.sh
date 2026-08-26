export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS="-j1 -l2"

colcon build --parallel-workers 1 --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF \
      -DLIGHTNING_WITH_PANGOLIN=OFF
