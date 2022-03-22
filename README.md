# Vcommunicator

need to install ros-tf2-*
sudo apt install ros-version-tf2-*

serial library for roscpp
https://github.com/wjwwood/serial.git

after 

$sudo systemctl stop nvgetty

$sudo systemctl disable nvgetty

$sudo systemctl stop 

$systemctl disable serial-getty@ttyS0

$udevadm trigger

$reboot

$sudo adduser $USER dialout
