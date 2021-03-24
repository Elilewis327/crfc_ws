# -a all : removes all build folders

while getopts a: flag
do
        case "${flag}" in
                a) a=${OPTARG};;
        esac
done

rm -rf build install

if [ "$a" = 'all' ] ; then

rm -rf build_aarch64 install_aarch64
fi