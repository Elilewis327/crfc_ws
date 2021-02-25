# Build before deploying

./scripts/cross_build.sh

docker save calvinrobotics/crfc2021:aarch64 | gzip > calvin_crfc.tar.gz