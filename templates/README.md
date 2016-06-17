<a id="top"/> 
#repository_name

**Description of the repository, for example:** This repository holds packages for hardware launch files and configuration, as well as the simulation model for starting up the basic layer for operating Robotino

Technical Maintainer: [**github-user**](https://github.com/**github-user**/) (**Maintainer's Name**, **Institution**) - **email@email.address.com**

##Contents

1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--execution">Execution</a>
3. <a href="#3--software-architecture">Software architecture</a>


## 1. Installation Requirements: <a id="1--installation-requirements"/> 

####Debian packages
**Only if requires a special debian package that has to be extra installed**

####Squirrel packages
**Only if requires other squirrel packages overlays. For example squirrel_common**

####ROS packages
**Commonly**
The ROS packages dependencies can be installed with the command:
```
rosdep install --from-path **repository_name** -i -y
```
## 2. Execution: <a id="2--execution"/> 
```
roslaunch **package_name file.launch**
```
**If needed, description of the parameters**

## 3. Software architecture <a id="3--software-architecture"/> 

**node name**: ![**node_name**](https://github.com/squirrel-project/**PackageName**/blob/indigo_dev/nade_name.png "Architecture")

<a href="#top">top</a>
