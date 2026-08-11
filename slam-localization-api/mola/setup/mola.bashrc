source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=0
source "$HOME/mola_ws/install/setup.bash"
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root="$HOME/mola_ws"
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

echo "=============================="
echo " MOLA environment"
echo "=============================="
echo "ROS_DISTRO=$ROS_DISTRO"
