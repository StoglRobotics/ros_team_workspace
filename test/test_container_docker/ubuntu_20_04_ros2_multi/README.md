# Docker Container ubuntu_20_04_ros2_multi

Ubuntu 20.04 with  ROS 2 and 
* [Foxy Fitzroy (codename ‘foxy’)](https://docs.ros.org/en/foxy/index.html)
* [Galactic Geochelone (codename ‘galactic’)](https://docs.ros.org/en/galactic/index.html)
* [Rolling Ridley (codename ‘rolling’)](https://docs.ros.org/en/rolling/index.html)  
installed with x11 forwading.

## container
The container should support forwarding of the x11-session. If this is not working please open an issue on github.  
The container itself should clone your `user` inside the docker container. The _ros_team_workspace_ folder is then shared with the docker container and mounted under _~/workspace/ros_team_workspace/_ inside the container.

## Usage of the docker container
Short:
1. Install docker
2. run `./build.sh`
3. run `./create.sh`
4. You can then connect either as root or user by opening a new terminal and run `:/connect_<user|root>.sh`.
5. If you want to exit simply type `exit` in the console inside the container. 
6. If you stop the container with `./stop.sh` or by exiting all instance restart with `./start.sh`.

### Install docker.
This step depends on the operatingsystem you are using. For instructions have a look [here](https://docs.docker.com/) on the official docs site or google it.
* [Windows](https://docs.docker.com/desktop/windows/install/)
* [Mac](https://docs.docker.com/desktop/mac/install/)
* Linux
    - have to look it up. However make sure your user is in the docker group. Check with: `groups`.  
    To add your user to the docker group run: `sudo usermod -aG docker <username>`. 



## some useful docker commands
For complete list of commands have a look at [official docker cli reference](https://docs.docker.com/engine/reference/commandline/cli/).

+ `docker container <command>`  
    + `ls` lists all current active containers
    + `ls -a` lists all containers
    + `rm <container>` removes container
+ `docker image <command>`   
    + `ls` lists all images
    + `rm <image>` removes image  
