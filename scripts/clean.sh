# -a all : removes all build folders

while getopts a: flag
do
        case "${flag}" in
                a) a=${OPTARG};;
        esac
done

sudo rm -rf build install

if [ "$a" = 'all' ] ; then

sudo rm -rf cc_internals build_aarch64 build_x86_64 install_aarch64 install_x86_64
fi