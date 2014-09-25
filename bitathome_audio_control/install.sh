#!/bin/bash

sudo apt-get install ros-indigo-sound-play festival festlex-cmu festlex-poslex festlex-oald libestools2.1 unzip

mkdir ~/cmu_tmp

cd ~/cmu_tmp

wget -c http://www.speech.cs.cmu.edu/cmu_arctic/packed/cmu_us_jmk_arctic-0.95-release.tar.bz2

echo "unpackaging..."
tar -jx -f cmu_us_jmk_arctic-0.95-release.tar.bz2
rm cmu_us_jmk_arctic-0.95-release.tar.bz2

echo "installing..."
sudo mkdir -p /usr/share/festival/voices/english/
sudo mv * /usr/share/festival/voices/english/
sudo mv /usr/share/festival/voices/english/cmu_us_jmk_arctic /usr/share/festival/voices/english/cmu_us_jmk_arctic_clunits

echo "clearing..."
cd ../
rm -rf cmu_tmp/

echo "done."
