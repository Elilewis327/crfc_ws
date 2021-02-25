#sudo ros_cross_compile $PWD -a aarch64 -d foxy -o ubuntu --runtime-tag calvinrobotics/crfc2021:aarch64

# colcon build --build-base ./build_aarch64 --install-base ./install_aarch64 --cmake-args -DCMAKE_TOOLCHAIN_FILE=$PWD/aarch64_toolchain.cmake -DCMAKE_CROSS_COMPILE_PREFIX=$PWD/aarch64/

colcon build --merge-install --cmake-force-configure --build-base ./build_aarch64 --install-base ./install_aarch64 --cmake-args -DCMAKE_TOOLCHAIN_FILE=~/crfc_ws/aarch64_toolchain.cmake -DCMAKE_CROSS_COMPILE_PREFIX=~/aarch64/opt/ros/foxy/
