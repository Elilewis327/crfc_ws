# Environment source
source /opt/ros/foxy/setup.$(basename $SHELL)

# Build and source
colcon build
source ./install/local_setup.$(basename $SHELL)