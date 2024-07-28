
pushd ~/alpha/
catkin_make -DCMAKE_BUILD_TYPE=Debug
cp -r build/driver devel/lib/
popd

source ~/.bashrc
roslaunch driver driver.launch
