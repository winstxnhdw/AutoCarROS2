#!/bin/bash

locale
read -p "Do you want to update your locale to en_SG.UTF-8 (y/n)?" input
if ["$input" = "y"];
    then
        sudo apt update
        sudo apt install locales
        sudo locale-gen en_SG en_SG.UTF-8
        sudo update-locale LC_ALL=en_SG.UTF-8 LANG=en_SG.UTF-8
        export LANG=en_SG.UTF-8
        locale

else
    echo "Proceeding without updating locale."

fi

sudo apt update
sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install python3-colcon-common-extensions
sudo apt install ros-foxy-desktop
sh requirements.sh
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc
