cd ~/

sudo apt update && sudo apt upgrade -y && sudo apt install git-lfs -y

# Install realsense
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# probably needs to be installed outside docker container?
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u

sudo apt-get install librealsense2-dkms -y
sudo apt-get install librealsense2-utils -y

# Setup Fan Control
git clone https://github.com/Pyrestone/jetson-fan-ctl.git
cd jetson-fan-ctl
sudo ./install.sh
cd ..

# start docker container (pull if necessary)
sudo usermod -aG docker $USER
sudo chown -R $USER ~/.docker

cd ~/crfc_ws
mkdir ~/crfc-vol/
git lfs pull
tar -xf install_aarch64.tar.gz -C ~/crfc-vol

docker run --restart always -d --network=host --name=crfc -v ~/crfc-vol:/home/ros/crfc-vol/:ro calvinrobotics/crfc2021:jetson
# Competition
# --restart always -d

# Testing
# --rm -it /bin/bash

# Clean up Jetson
sudo rm -rf /home/nvidia/cudnn /home/nvidia/OpenCV /home/nvidia/TensorRT /home/nvidia/libvisionworkd*
# Save ~400MB
sudo apt remove --purge -y thunderbird libreoffice-* unattended-upgrade

# Setup Wifi and ethernet
#sudo echo "up route add -net 10.18.76.$id netmask 255.255.255.0 gw 10.18.76.1 eth0" >> /etc/network/interfaces