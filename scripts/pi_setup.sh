# ./scripts/pi_setup -i 10 (IP #)

while getopts i: flag
do
        case "${flag}" in
                i) id=${OPTARG};;
        esac
done

# Update package list
sudo apt-get update && sudo apt-get upgrade -y

# Install Docker
sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

sudo add-apt-repository \
   "deb [arch=arm64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"

sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io -y
sudo usermod -aG docker ubuntu

echo \
"network:
    ethernets:
        eth0:
            dhcp4: false
            addresses: [10.18.76.$id/8]
            gateway4: 10.18.76.1
            nameservers:
                addresses: [8.8.8.8,8.8.4.4,10.18.76.1]

    version: 2
    wifis:
        wlan0:
            access-points:
                'calvin-crfc':
                    password: 'calvin-crfc'
            dhcp4: false
            optional: true
            addresses: [10.18.76.$id/24]
            gateway4: 10.18.76.1
            nameservers:
                addresses: [8.8.8.8,8.8.4.4,10.18.76.1]" \
| sudo tee /etc/netplan/50-cloud-init.yaml


sudo netplan generate
sudo netplan apply
ip a

