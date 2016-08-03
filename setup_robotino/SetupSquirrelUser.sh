#!/bin/bash
#### COMMON PARAMETERS
usage=$(cat <<"EOF"
run as user robotino
USAGE:  ./$0\n
EOF
)
green='\e[0;32m'
red='\e[0;31m'
NC='\e[0m' # No Color


#### FUNCTION check if sudo can be executed 
function CheckUserSudo {
  if [[ `whoami` != 'robotino' ]]
    then
      echo -e "\n${red}FAILURE: Not running as user 'robotino'${NC}\n"
      exit 1
  fi
  sudo -v
  if [[ $? != 0 ]]
    then 
      echo -e "\n${red}FAILURE: robotino is not able to run sudo ${NC}\n"
      exit 1
  fi
}


#### FUNCTION CREATE squirrel USER
function CreateSquirrelUser{
  echo -e "\n${green}INFO: Create user 'squirrel'${NC}\n"
  if [[sudo adduser squirrel
  if [[ $? != 0 ]]
    then 
      echo -e "\n${red}FAILURE: Unable to create user 'squirrel'${NC}\n"
      exit 1
  fi
  echo -e "\n${green}INFO: Add user 'squirrel' to group 'sudo'${NC}\n"
  sudo usermod -aG sudo squirrel
  if [[ $? != 0 ]]
    then 
      echo -e "\n${red}FAILURE: Unable to add squirrel to sudo ${NC}\n"
      exit 1
  fi
  echo "# Allow members of group sudo to execute any command" | sudo tee -a /etc/sudoers
  echo "%sudo   ALL=(ALL:ALL) ALL" | sudo tee -a /etc/sudoers

}

########################################################################
############################# INITIAL MENU #############################
########################################################################

CheckUserSudo
CreateSquirrelUser
