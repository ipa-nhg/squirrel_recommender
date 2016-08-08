#!/bin/bash
green='\e[0;32m'
red='\e[0;31m'
NC='\e[0m' # No Color

#### FUNCTION check if sudo can be executed 
function CheckUserSudo () {
  echo -e "\n${green}CheckUserSudo got $1 as argument${NC}\n"
  if [[ `whoami` -ne $1 ]]
    then
      echo -e "\n${red}FAILURE: Not running as user '$1'${NC}\n"
      exit 1
  fi
  echo -e "\n${green}INFO: Check if user $1 is able to run commands with sudo${NC}\n"
  sudo -v
  if [[ $? != 0 ]]
    then 
      echo -e "\n${red}FAILURE: $1 is not able to run sudo ${NC}\n"
      exit 1
  fi
  echo -e "\n${green}INFO: Running as user `whoami` in directory `pwd`${NC}\n"
}

CheckUserSudo squirrel

echo -e "\n${green}INFO: Checkout the recommender repository${NC}\n"
sleep 5
git clone git://github.com/squirrel-project/squirrel_recommender.git

echo -e "\n${green}INFO: Setup udev rules${NC}\n"
sleep 5
sudo cp /home/squirrel/squirrel_recommender/setup_robotino/udev_rules/* /etc/udev/rules.d/.
sudo udevadm control --reload-rules

echo -e "\n${green}INFO: Setup bash environment${NC}\n"
sleep 5
sudo cp /home/squirrel/squirrel_recommender/setup_robotino/robotino.bash.bashrc /etc/robotino.bash.bashrc

echo -e "\n${green}INFO: Setup bash environment${NC}\n"
sleep 5
cp /home/squirrel/squirrel_recommender/setup_robotino/user.bashrc /home/squirrel/.bashrc
source /opt/ros/indigo/setup.bash
sudo sed -i "s/myrobot/$ROBOT/g" /home/squirrel/.bashrc
sudo sed -i "s/mydistro/$ROS_DISTRO/g" /home/squirrel/.bashrc
sudo sed -i "s/witharm/$ARM/g" /home/squirrel/.bashrc

echo -e "\n${green}INFO: Create overlays for stacks${NC}\n"
sleep 5
mkdir /home/squirrel/catkin_ws
mkdir /home/squirrel/catkin_ws/src
cd /home/squirrel/catkin_ws/src
source /opt/ros/indigo/setup.bash
catkin_init_workspace
cd /home/squirrel/catkin_ws
catkin_make
cd /home/squirrel/catkin_ws/src
git clone https://github.com/squirrel-project/squirrel_robotino
git clone https://github.com/squirrel-project/squirrel_common
git clone https://github.com/squirrel-project/squirrel_driver
git clone https://github.com/squirrel-project/squirrel_kclhand
git clone https://github.com/squirrel-project/squirrel_robotino_arm
cd /home/squirrel/catkin_ws/
catkin_make install
rosdep update
rosdep install --from-path src -i -y


echo -e "\n${green}INFO: Enable passwordless login${NC}\n"
sleep 5
ssh-keygen
cat /home/squirrel/.ssh/id_rsa.pub | ssh squirrel@robotino "cat >> /home/squirrel/.ssh/authorized_keys"



