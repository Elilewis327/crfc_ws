# Source Environment
# For Jetsons and Pis: aarch64
# source ~/aarch64/opt/ros/setup.$(basename $SHELL)
if [ ! -d ~/aarch64 ]; then mkdir ~/aarch64 && tar -xf /workspaces/crfc_ws/cross-aarch64.tar.gz -C ~/aarch64 --strip-components=3
fi
# Alternate using official cross-compile
#sudo ros_cross_compile $PWD -a aarch64 -d foxy -o ubuntu --runtime-tag calvinrobotics/crfc2021:aarch64
source ~/aarch64/opt/ros/foxy/setup.bash

colcon build --merge-install --cmake-force-configure --build-base ./build_aarch64 --install-base ./install_aarch64 --cmake-args -DCMAKE_TOOLCHAIN_FILE=/workspaces/crfc_ws/aarch64_toolchain.cmake -DCMAKE_CROSS_COMPILE_PREFIX=~/aarch64/opt/ros/foxy/

# Jetson setup file
cp ./install_aarch64/setup.bash ./install_aarch64/jetson_setup.bash
sed -i "s:/home/ros/aarch64::g" ./install_aarch64/jetson_setup.bash
sed -i "s:foxy:foxy/install:g" ./install_aarch64/jetson_setup.bash
sed -i "s:/workspaces/crfc_ws/install:/home/ros/crfc-vol/install_aarch64:g" ./install_aarch64/jetson_setup.bash

# Change location of setup files from cross build -> native
sed -i "s:/home/ros/aarch64::g" ./install_aarch64/setup.bash
sed -i "s:/workspaces/crfc_ws/install:/home/ros/crfc-vol/install_aarch64:g" ./install_aarch64/setup.bash