#!/bin/sh

#openni_launch
sudo apt-get install ros-indigo-openni-launch
#glut
sudo apt-get install libghc-glut-*
#nite
wget "http://pan.bitathome.org/public.php?service=files&t=de7f3c744c49b2a2c6bc45760a35dd57&download" -O nite.tar.bz2
tar -xjv -f nite.tar.bz2
cd NITE-Bin-Dev-Linux-x64-v1.5.2.21
sudo ./install.sh
cd ..
rm NITE-Bin-Dev-Linux-x64-v1.5.2.21/ -rf
mv nite.tar.bz2 ~/Downloads/
#make workspace
mkdir catkin_dependence
cd catkin_dependence
mkdir src
cd src
catkin_init_workspace
#get code
git clone https://github.com/ros-drivers/openni_tracker.git
git clone https://github.com/pirobot/skeleton_markers.git
cd skeleton_markers
git checkout hydro-devel
cd ../..
#make
catkin_make

