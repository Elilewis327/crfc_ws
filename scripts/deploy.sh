# Build before deploying
./scripts/cross_build.sh

# Compress build output
tar -czf install_aarch64.tar.gz ./install_aarch64 ./launch

# Send folder to each target: specified by json
i=0

while [ $i -lt $(jq "length" deployment.json) ]
do

ADDRESS=$(jq -r ".[$i].ip" deployment.json)
NAME=$(jq -r ".[$i].name" deployment.json)
PSWD=$(jq -r ".[$i].password" deployment.json)

# check if host is reachable
if ping -c 1 10.18.76.$ADDRESS &> /dev/null
then
  # Docker:Dev -> Target
  sshpass -p $PSWD scp -r install_aarch64.tar.gz $NAME@10.18.76.$ADDRESS:~/
  # SSH: copy into container, restart docker container: ends current ros2 nodes, and restarts updated ones
  sshpass -p $PSWD ssh $NAME@10.18.76.$ADDRESS "tar -xf ~/install_aarch64.tar.gz -C ~/crfc-vol/"

  sshpass -p $PSWD ssh $NAME@10.18.76.$ADDRESS "rm ~/install_aarch64.tar.gz | docker container restart -t 0 crfc"

  echo "Successfully deployed to: 10.18.76.$ADDRESS"
else
  echo "10.18.76.$ADDRESS is unreachable"
fi

((++i))
done