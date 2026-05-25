#!/bin/env bash
set -e

version=0.3.0
arch=arm64

echo "building deb for blackice_traffic $version ($arch)"

if ! type "dpkg-deb" > /dev/null; then
  echo "please install required build tools first"
  exit 1
fi

project="blackice-traffic_${version}_${arch}"
folder_name="build/$project"
echo "creating $folder_name"
mkdir -p $folder_name
cp -r DEBIAN/ $folder_name
bin_dir="$folder_name/usr/bin"
lib_dir="$folder_name/usr/lib/blackice_traffic"
res_dir="$lib_dir/resources"
mkdir -p $bin_dir
mkdir -p $lib_dir
mkdir -p $res_dir
cp blackice_traffic $bin_dir
cp resources/icon.ico $res_dir
cp resources/icon.png $lib_dir
cp resources/blackice_traffic.desktop $lib_dir
cp LICENSE $lib_dir
cp *.py $lib_dir
cp GeoLite2-City.mmdb $lib_dir

sed -i "s/_version_/$version/g" $folder_name/DEBIAN/control
sed -i "s/Architecture: amd64/Architecture: $arch/g" $folder_name/DEBIAN/control
cd build/ && dpkg-deb --build -Z gzip --root-owner-group $project
