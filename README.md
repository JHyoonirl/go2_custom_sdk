# go2_custom_sdk
unitree go2 visualization code


## Installation

'''shell
mkdir -p ros2_ws
cd ros2_ws
git clone --recurse-submodules https://github.com/JHyoonirl/go2_custom_sdk.git src
sudo apt install ros-$ROS_DISTRO-image-tools
sudo apt install ros-$ROS_DISTRO-vision-msgs

cd src
pip install -r requirements.txt
cd ..
'''