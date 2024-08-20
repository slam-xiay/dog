sudo ifconfig eno1 192.168.3.200 netmask 255.255.255.0
sudo rm -rf /dev/imu
sudo cp ../udev/imu.rules /etc/udev/rules.d/
sudo udevadm trigger

pushd ~/alpha/
catkin_make -DCMAKE_BUILD_TYPE=Debug -DCAKIN_ENABLE_TESTING=OFF -DCATKIN_WHITELIST_PACKAGES=driver
cp -r build/driver devel/lib/
popd

source ~/.bashrc
roslaunch driver driver.launch
