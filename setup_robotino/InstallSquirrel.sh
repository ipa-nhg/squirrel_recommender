#!/bin/bash
#### COMMON PARAMETERS
usage=$(cat <<"EOF"
USAGE:  ./InstallSquirrel.sh [-r] [-arm]\n
Where:\n
  -r robot name (example alufr-robotino)\n
  -arm true or false\n

EOF
)
green='\e[0;32m'
red='\e[0;31m'
NC='\e[0m' # No Color
DISTRIB_CODENAME='trusty'


#### FUNCTION check if sudo can be executed 
function CheckUserSudo {
  if [[ `whoami` != 'squirrel' ]]
    then
      echo -e "\n${red}FAILURE: Not running as user 'squirrel'${NC}\n"
      exit 1
  fi
  sudo -v
  if [[ $? != 0 ]]
    then 
      echo -e "\n${red}FAILURE: Squirrel is not able to run sudo ${NC}\n"
      exit 1
  fi
}


#### FUNCTION ROS INSTALLATION
function RosInstallation {
  echo -e "\n${green}INFO:Setup ROS${NC}\n"
  sudo sh -c '. /etc/lsb-release && echo "deb http://packages.ros.org.ros.informatik.uni-freiburg.de/ros/ubuntu $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
  sudo apt-get update
  sudo apt-get install ros-indigo-ros-base
  sudo rosdep init
  
}

#### FUNCTION BASIC INSTALLATION
function BasicInstallation {

  echo -e "\n${green}INFO:Installing basic tools${NC}\n"
  sleep 5
  sudo apt-get update
  sudo apt-get install vim tree gitg meld curl openjdk-6-jdk zsh terminator language-pack-de language-pack-en ipython htop python-pip -y --force-yes

  echo -e "\n${green}INFO:Update grub to avoid hangs on reboot${NC}\n"
  sleep 5
  if grep -q GRUB_RECORDFAIL_TIMEOUT= /etc/default/grub ; then
    echo "found GRUB_RECORD_FAIL flag already, skipping update-grub call"
  else
    echo GRUB_RECORDFAIL_TIMEOUT=10 | sudo tee -a /etc/default/grub
    sudo update-grub
  fi


  echo -e "\n${green}INFO:Install openssh server${NC}\n"
  sleep 5
  sudo apt-get install openssh-server -y --force-yes
  echo -e "\n${green}INFO:Let the server send a alive interval to clients to not get a broken pipe${NC}\n"
  echo "ClientAliveInterval 60" | sudo tee -a /etc/ssh/sshd_config
  sudo sed -i 's/PermitRootLogin without-password/PermitRootLogin yes/g' /etc/ssh/sshd_config

  echo -e "\n${green}INFO:Checkout the recommender repository${NC}\n"
  sleep 5
  git clone git://github.com/squirrel-project/squirrel_recommender.git

  echo -e "\n${green}INFO:Allow squirrel user to execute sudo command without password${NC}\n"
  sleep 5
 
  echo "squirrel ALL=(ALL) NOPASSWD: ALL" | sudo tee -a /etc/sudoers
  sudo adduser squirrel dialout
  sudo adduser squirrel audio
  sudo adduser squirrel pulse
    

  echo -e "\n${green}INFO: Setup udev rules${NC}\n"
  sleep 5
  sudo cp ~/squirrel_recommender/setup_robotino/udev_rules/* /etc/udev/rules.d/.
  sudo udevadm control --reload-rules


  echo -e "\n${green}INFO: Setup bash environment${NC}\n"
  sleep 5
  sudo cp /home/squirrel/squirrel_recommender/setup_robotino/robotino.bash.bashrc /etc/robotino.bash.bashrc

  echo -e "\n${green}INFO:  Setup bash environment${NC}\n"
  sleep 5
  cp /home/squirrel/squirrel_recommender/setup_robotino/user.bashrc /home/squirrel/.bashrc
  source /opt/ros/indigo/setup.bash
  sudo sed -i "s/myrobot/$ROBOT/g" /home/squirrel/.bashrc
  sudo sed -i "s/mydistro/$ROS_DISTRO/g" /home/squirrel/.bashrc
  sudo sed -i "s/witharm/$ARM/g" /home/squirrel/.bashrc
  
  echo -e "\n${green}INFO:  Create overlays for stacks${NC}\n"
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
  
  
  echo -e "\n${green}INFO:  Enable passwordless login${NC}\n"
  sleep 5
  ssh-keygen
  cat /home/squirrel/.ssh/id_rsa.pub | ssh squirrel@robotino "cat >> /home/squirrel/.ssh/authorized_keys"


}

#### UpstartInstallation
function UpstartInstallation {
  sudo apt-get install ros-indigo-robot-upstart
  sudo cp /home/squirrel/squirrel_recommender/setup_robotino/upstart/robotino.conf /etc/init/robotino.conf
  sudo cp /home/squirrel/squirrel_recommender/setup_robotino/upstart/robotino-start /usr/sbin/robotino-start
  sudo sed -i "s/myrobot/$ROBOT/g" /usr/sbin/robotino-start
  sudo sed -i "s/mydistro/$ROS_DISTRO/g" /usr/sbin/robotino-start
  sudo sed -i "s/witharm/$ARM/g" /usr/sbin/robotino-start
  sudo cp /home/squirrel/squirrel_recommender/setup_robotino/upstart/robotino-stop /usr/sbin/robotino-stop
	

  sudo mkdir -p /etc/ros/$ROS_DISTRO/robotino.d
	sudo ln -s /home/squirrel/squirrel_recommender/setup_robotino/upstart/robotino.d/setup /etc/ros/$ROS_DISTRO/robotino.d/setup
  sudo cp -rf /home/squirrel/squirrel_recommender/setup_robotino/upstart/robotino.d/launch /etc/ros/$ROS_DISTRO/robotino.d/
  sudo sed -i "s/myrobot/$ROBOT/g" /etc/ros/$ROS_DISTRO/robotino.d/launch/robot/robot.launch

  echo -e "\n${green}INFO:  Define users rights${NC}\n"
  sleep 5
  sudo echo "%users ALL=NOPASSWD:/usr/sbin/robotino-start" | sudo tee -a /etc/sudoers
  sudo echo "%users ALL=NOPASSWD:/usr/sbin/robotino-stop" | sudo tee -a /etc/sudoers
}
########################################################################
############################# INITIAL MENU #############################
########################################################################


if [[ "$@" =~ "--help" || $# < 2 ]]; then echo -e $usage; exit 0; fi

while [[ $# >=1 ]]
do
key="$1"
shift
case $key in
    -r|--robot)
    ROBOT="$1"
    shift
    ;;
    -arm|--arm)
    ARM="$1"
    shift
    ;;
esac
done

if [ -z "$ROBOT" ]; then
    echo -e $usage
    exit
fi

if [ -z "$ARM" ]; then
    echo -e $usage
    exit
fi


echo -e "\n${green}===========================================${NC}\n"
echo -e ROBOT  = "${green}${ROBOT}${NC}"
echo -e ARM     = "${green}${ARM}${NC}"
echo -e "\n${green}===========================================${NC}\n"
read -p "Continue (y/n)?" choice
case "$choice" in 
  y|Y ) ;;
  n|N ) exit;;
  * ) echo "invalid" && exit 1;;
esac

CheckUserSudo      

read -p "Please select the installation type 
0. ROS installation
1. Basic installation
2. Upstart Installation

 " choice 
 
if [[ "$choice" == 0 ]]
  then
       RosInstallation
fi

if [[ "$choice" == 1 ]]
  then
       BasicInstallation
fi

if [[ "$choice" == 2 ]]
  then
       UpstartInstallation
fi
