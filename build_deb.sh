#!/usr/bin/env bash
set -euo pipefail

arch="${1:-amd64}"
version="$(sed -n 's/^APP_VERSION *= *"\(.*\)".*/\1/p' blackice_traffic.py)"

if [ -z "$version" ]; then
  echo "ERROR: could not read APP_VERSION from blackice_traffic.py"
  exit 1
fi

echo "building deb for blackice_traffic $version ($arch)"

if ! command -v dpkg-deb >/dev/null 2>&1; then
  echo "ERROR: dpkg-deb not found. Install build tools: sudo apt-get install dpkg-dev"
  exit 1
fi

if [ ! -f blackice_traffic ]; then
  echo "ERROR: ./blackice_traffic binary not found. Run ./build_linux_bin.sh (or make bin) first."
  exit 1
fi

echo "running core test (syntax check)"
python3 -m py_compile blackice_traffic.py

project="blackice-traffic_${version}_${arch}"
folder_name="build/$project"
echo "creating $folder_name"
rm -rf "$folder_name"
mkdir -p "$folder_name"
cp -r DEBIAN/ "$folder_name"
bin_dir="$folder_name/usr/bin"
lib_dir="$folder_name/usr/lib/blackice_traffic"
res_dir="$lib_dir/resources"
mkdir -p "$bin_dir" "$lib_dir" "$res_dir"
cp blackice_traffic "$bin_dir"
cp resources/icon.ico "$res_dir"
cp resources/icon.png "$lib_dir"
cp resources/blackice_traffic.desktop "$lib_dir"
cp LICENSE "$lib_dir"
cp GeoLite2-City.mmdb "$lib_dir"

sed -i "s/_version_/$version/g" "$folder_name/DEBIAN/control"
if [ "$arch" != "amd64" ]; then
  sed -i "s/Architecture: amd64/Architecture: $arch/g" "$folder_name/DEBIAN/control"
fi

cd build/ && dpkg-deb --build -Z gzip --root-owner-group "$project"
