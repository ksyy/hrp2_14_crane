hrp2_14_crane
=============

ROS package to control the HRP-2 14 crane


Building
--------
mkdir _build-RELEASE
cd _build-RELEASE
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
make

In the source directory, you should find the directory bin
which contains the 
crane_listener 
executable.

This executable gives the position of the crane.


