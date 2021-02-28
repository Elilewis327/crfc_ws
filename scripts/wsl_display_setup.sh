# use it to export DISPLAY into WSL2 

cd ~

echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2; exit;}'):0.0
export LIBGL_ALWAYS_INDIRECT=0
sudo /etc/init.d/dbus start &> /dev/null" >> ".$(basename $SHELL)rc"

# Re-source environment
source ".$(basename $SHELL)rc"
