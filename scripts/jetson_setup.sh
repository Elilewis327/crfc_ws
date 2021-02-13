while getopts i: flag
do
        case "${flag}" in
                i) id=${OPTARG};;
        esac
done

# Testing
#echo "-net 10.18.76.$id";

cd ~/

sudo apt update && apt upgrade

# Install realsense
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# Check if bionic or focal: probably needs to be installed outside docker container, so bionic?
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
#sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo focal main" -u

sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

# Setup Fan Control
git clone https://github.com/Pyrestone/jetson-fan-ctl.git
cd jetson-fan-ctl
sudo ./install.sh
cd ..


# Clean up Jetson
sudo rm -rf /home/nvidia/cudnn /home/nvidia/OpenCV /home/nvidia/TensorRT /home/nvidia/libvisionworkd*
# Save ~400MB
sudo apt remove --purge -y thunderbird libreoffice-* unattended-upgrade

# Setup Wifi and ethernet
echo "up route add -net 10.18.76.$id netmask 255.255.255.0 gw 10.18.76.1 eth0" >> /etc/network/interfaces