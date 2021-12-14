# Usage of the docker container
Short:
1. Install docker
2. run `./build.sh`
3. run `./create.sh`
4. You can then connect either as root or user by opening a new terminal and run `:/connect_<user|root>.sh`.
5. If you want to exit simply type `exit` in the console inside the container. 
6. If you stop the container with `./stop.sh` or by exiting all instance restart with `./start.sh`.

## Install docker.
This step depends on the operatingsystem you are using. For instructions have a look [here](https://docs.docker.com/) on the official docs site or google it.
* [Windows](https://docs.docker.com/desktop/windows/install/)
* [Mac](https://docs.docker.com/desktop/mac/install/)
* Linux
    - have to look it up. However make sure your user is in the docker group. Check with: `groups`. To add your user to the docker group run: `sudo usermod -aG docker <username>`. 

## container
TODO
