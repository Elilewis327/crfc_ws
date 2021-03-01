# Source Environment
# source ~/aarch64/opt/ros/setup.$(basename $SHELL)
if [ ! -d ${WORKSPACE}/../aarch64 ]; then mkdir ${WORKSPACE}/../aarch64 && tar -xf ${WORKSPACE}/cross-aarch64.tar.gz -C ${WORKSPACE}/../aarch64 --strip-components=1
# Alternate using official cross-compile
#sudo ros_cross_compile $PWD -a aarch64 -d foxy -o ubuntu --runtime-tag calvinrobotics/crfc2021:aarch64

colcon build --merge-install --cmake-force-configure --build-base ./build_aarch64 --install-base ./install_aarch64 --cmake-args -DCMAKE_TOOLCHAIN_FILE=~/crfc_ws/aarch64_toolchain.cmake -DCMAKE_CROSS_COMPILE_PREFIX=~/aarch64/opt/ros/foxy/
