# squirrel_recommender

### Contents

1. <a href="#Robotino-manual">Robotino manual</a>
     1. <a href="#Administrator-manual">Administrator Manual</a>
     2. <a href="#User-manual">User Manual</a>
2. <a href="#Documentation-and-Templates">Documentation and Templates</a>
     1. <a href="#Software-architecture">Software architecture</a>
     2. <a href="#Readme">Readme</a>
3. <a href="#Hardware-test">Hardware Tests</a>


### 1. Robotino manual <a id="1--installation-requirements"/> 
This section provide some instructions to install a standard setup.
 
#### 1.1. Administrator Manual <a id="Administrator-manual"/> 
##### Installation
The first step is create a new user "squirrel":
```
sudo adduser squirrel
sudo adduser squirrel sudo 
```
Login with your new user "squirrel" and install the setup. The setup of the robot can be done using a bash script. The script is in the squirrel_recommender repository, you can get it using the following command:
```
wget https://raw.githubusercontent.com/squirrel-project/squirrel_recommender/master/setup_robotino/InstallSquirrel.sh
chmod +x InstallSquirrel.sh
```

The installation script needs the parameters robot and arm, where:

* -r robot: is the robot name (ipa-robotino)
* -arm : should be true if your robot has an arm and false if it doesn't

The script allow two types of installation:

* Basic Installation. This part will install some basic tools, setup your new squirrel user, clone the necessary repositories, configure the user rights and create the udev rules.
* Upstart Installation. This installation will create a startup job, your robot will start automatically the low level controls after switch on the platform. 

##### New users

When the installation process finish you will have on your robot a new user, with a stable version of the bringup level drivers. This user will be used by the upstart job to init your robot, and for other users as reference. Please **NEVER** use the "squirrel" user to develop or test new code.

Each person who wants to use the robot should have an account , an accound can be created by the "squirrel" user with the following command:
```
sudo robotino-adduser *UserName*
```
This user will be automatically configured, getting the necessary rights to start and stop the robot and a new preconfigured catkin workspace under */home/UserName/catkin_ws*. By default the *ROS_PACKAGE_PATH* of this user will be the following:
```
/home/UserName/catkin_ws/src:/home/squirrel/catkin_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks
```
That means, if ROS needs to find a package, first will check if my UserName has an Overlay, if not will check if squirrel user has an Overlay and as last option will take the ROS release. 
For example, if I want to test the actual status of the perception. I can create a new account "perception", and clone into */home/perception/catkin_ws/src* the perception repository, compile only the perception repository and run it. All the messages and common software package needed are already on "squirrel" user compiled. 

#### 1.2. User Manual <a id="User-manual"/> 
##### Start and stop the robot

Automatically and by default (if you executed the upstart installation) the bringup drivers of the robot will be started on Boot. This process will run as a background job, calling the arm server, waiting until the arm is ready and starting the robot.launch. Usually the robot with arm will require some minutes to start up, you can hear a "Click" when the drivers are already launched and check with a *rostopic list* command if the launch file was started.

In case that you need to stop the robot drivers you can call the command *sudo robotino-stop* , all the users have rights to call this command as sudo. If you want to start again the low level control of the robot , you can call the command *sudo robotino_start*. These commands use the "squirrel" user software status, please be sure that this user has always a working version.

### 2. Documentation and Templates <a id="Documentation-and-Templates"/> 

#### 2.1. Software architecture <a id="Software-architecture"/> 

To generate your own software architecture diagram:

* open the website [https://www.draw.io/](https://www.draw.io/)
* Choose *Create New Diagram*
* go to *file* -> *Import from* and choose URL
* copy the template URL : https://raw.githubusercontent.com/squirrel-project/squirrel_recommender/master/templates/architecture_template.xml
* after edit the template, export it as image and as xml 
* copy and commit the xml file to [squirrel_recommender/software_architecture](https://github.com/squirrel-project/squirrel_recommender/tree/master/software_architecture), to have a backup in case you need to modify the diagram.
* copy and commit the png file to your repository, the Readme will display the image.

#### 2.2. Readme <a id="Readme"/> 

Use the template under: [Readme Template](https://raw.githubusercontent.com/squirrel-project/squirrel_recommender/master/templates/README.md) to create your repository documentation. Please see [squirrel_robotino](https://github.com/squirrel-project/squirrel_robotino/blob/indigo_dev/README.md) as example.

### 3. Hardware Tests <a id="Hardware-test"/> 
