while getopts o: flag
do
        case "${flag}" in
                o) a=${OPTARG};;
        esac
done

# Update 
#sudo apt-get update && sudo apt-get upgrade -y

# Move to home directory 
cd ~
# Export x-server to default shell if running wsl
# add option: -o wsl
# zsh
if [ "$a" = 'wsl' ] ; then
# if doesn't work
# ["$a" == "wsl"]
echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2; exit;}'):0.0
export LIBGL_ALWAYS_INDIRECT=0
sudo /etc/init.d/dbus start &> /dev/null" >> ".$(basename $SHELL)rc"
fi

echo "export PATH=\$HOME/.local/bin:\$PATH" >> ".$(basename $SHELL)rc"


# Install ROS Noetic (for now until all packages are updated for ROS 2)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update
sudo apt install ros-noetic-desktop -y

sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update

# Install ROS Foxy
sudo apt update && sudo apt install curl gnupg2 lsb-release -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install -y ros-foxy-desktop ros-foxy-rqt ros-foxy-diagnostic-updater

sudo apt install -y python3-pip
sudo apt install -y python3-colcon-common-extensions
pip3 install -U argcomplete

# Install Cross Compile - requires docker on the system
sudo apt-get install qemu-user-static
pip3 install ros_cross_compile

# Save ~400MB
#sudo apt remove --purge -y thunderbird libreoffice-*