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
#ROS_MIRROR='http://packages.ros.org/ros/ubuntu'
ROS_MIRROR='http://packages.ros.org.ros.informatik.uni-freiburg.de/ros/ubuntu'

#### FUNCTION check if sudo can be executed 
function CheckUserSudo () {
  echo -e "\n${green}CheckUserSudo got $1 as argument${NC}\n"
  #if [[ $USER != $1 ]]
  if [[ `whoami` -ne $1 ]]
    then
      echo -e "\n${red}FAILURE: Not running as user '$1'${NC}\n"
      exit 1
  fi
  echo -e "\n${green}INFO: Check if user $1 is able to run commands with sudo${NC}\n"
  sleep 5 
  sudo -v
  if [[ $? != 0 ]]
    then 
      echo -e "\n${red}FAILURE: $1 is not able to run sudo ${NC}\n"
      exit 1
  fi
}

#### FUNCTION CREATE squirrel USER
function CreateSquirrelUser {
  ret=false
  getent passwd squirrel >/dev/null 2>&1 && ret=true
  
  if $ret; then
    echo -e "\n${green}INFO: User squirrel already exists${NC}\n"
  else
    echo -e "\n${green}INFO: Create user 'squirrel'${NC}\n"
    sudo adduser squirrel
    if [[ $? != 0 ]]
      then 
        echo -e "\n${red}FAILURE: Unable to create user 'squirrel'${NC}\n"
        exit 1
    fi
  fi

  for i in 'sudo' 'dialout' 'audio';
    do
      if groups squirrel | grep &>/dev/null $i; then
        echo -e "\n${green}INFO: User squirrel already in $i${NC}\n"
      else
        echo -e "\n${green}INFO: Add user 'squirrel' to group $i${NC}\n"
        sudo usermod -aG sudo squirrel
        if [[ $? != 0 ]]
          then 
            echo -e "\n${red}FAILURE: Unable to add squirrel to $i${NC}\n"
            exit 1
        fi
      fi
    done
  
  if sudo grep "squirrel ALL=(ALL) NOPASSWD: ALL" /etc/sudoers &> /dev/null
    then
      echo -e "\n${green}INFO: User squirrel is already in sudoers file${NC}\n"
  else
    echo -e "\n${green}INFO: Allow squirrel user to execute sudo command without password${NC}\n"
    echo "squirrel ALL=(ALL) NOPASSWD: ALL" | sudo tee -a /etc/sudoers &> /dev/null
  fi    
  if sudo grep "%sudo   ALL=(ALL:ALL) ALL" /etc/sudoers &> /dev/null
    then
      echo -e "\n${green}INFO: Group sudo is already in sudoers file${NC}\n"
  else
    echo -e "\n${green}INFO: Allow group sudo to execute sudo${NC}\n"
    echo "%sudo   ALL=(ALL:ALL) ALL" | sudo tee -a /etc/sudoers &> /dev/null
  fi    
}

#### FUNCTION ROS INSTALLATION
function RosInstallation {

  echo -e "\n${green}   INFO: Setup your source.list${NC}\n"
  echo -e "\n${green}   INFO: ROS_MIRROR is set to Freiburg University. Can be changed in InstallSquirrel.sh if needed${NC}\n"
  sudo sh -c '. /etc/lsb-release && echo "deb $ROS_MIRROR $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
  echo -e "\n${green}   INFO: Set up your keys${NC}\n"
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

  sudo apt-get update
  sudo apt-get install ros-indigo-ros-base -y --force-yes
  sudo rosdep init
  
}

#### FUNCTION BASIC INSTALLATION
function BasicInstallation {

  echo -e "\n${green}INFO: Installing basic tools${NC}\n"
  sleep 5
  sudo apt-get update
  sudo apt-get install vim tree gitg meld curl openjdk-6-jdk zsh terminator language-pack-de language-pack-en ipython htop python-pip -y --force-yes

  echo -e "\n${green}INFO: Update grub to avoid hangs on reboot${NC}\n"
  sleep 5
  if grep -q GRUB_RECORDFAIL_TIMEOUT= /etc/default/grub ; then
    echo "found GRUB_RECORD_FAIL flag already, skipping update-grub call"
  else
    echo GRUB_RECORDFAIL_TIMEOUT=10 | sudo tee -a /etc/default/grub &> /dev/null
    sudo update-grub
  fi


  echo -e "\n${green}INFO: Install openssh server${NC}\n"
  sleep 5
  sudo apt-get install openssh-server -y --force-yes
  echo -e "\n${green}INFO: Let the server send a alive interval to clients to not get a broken pipe${NC}\n"
  echo "ClientAliveInterval 60" | sudo tee -a /etc/ssh/sshd_config &> /dev/null
  sudo sed -i 's/PermitRootLogin without-password/PermitRootLogin yes/g' /etc/ssh/sshd_config
  sudo -u squirrel -i `pwd`/SquirrelSetup.sh

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

  echo -e "\n${green}INFO: Define users rights${NC}\n"
  sleep 5
  if sudo grep "%users ALL=NOPASSWD:/usr/sbin/robotino-start" /etc/sudoers &> /dev/null
    then
      echo -e ""
  else
    echo "%users ALL=NOPASSWD:/usr/sbin/robotino-start" | sudo tee -a /etc/sudoers &> /dev/null
  fi    

  if sudo grep "%users ALL=NOPASSWD:/usr/sbin/robotino-stop" /etc/sudoers &> /dev/null
    then
      echo -e ""
  else
    echo "%users ALL=NOPASSWD:/usr/sbin/robotino-stop" | sudo tee -a /etc/sudoers &> /dev/null
  fi    
}

function Main {
echo -e '\n################## Main Menu ##################\n'
read -p "Please select the installation type 
1. Create squirrel user 
2. ROS installation
3. Basic installation
4. Upstart Installation

q. Quit

 " choice 
 
if [[ "$choice" == 1 ]]
  then
       CreateSquirrelUser       
       Main
fi
if [[ "$choice" == 2 ]]
  then
       RosInstallation
       Main
fi

if [[ "$choice" == 3 ]]
  then
       BasicInstallation
       Main
fi

if [[ "$choice" == 4 ]]
  then
       UpstartInstallation
       Main
fi

if [[ "$choice" == 'q' ]]
  then
       exit 0 
fi
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

Main
