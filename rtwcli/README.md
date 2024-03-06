# rtw cli installation

`git clone https://github.com/StoglRobotics/ros_team_workspace.git`

`cd ros_team_workspace/rtwcli/ && pip3 install -r requirements.txt`

`source ros_team_workspace/setup.bash`

`setup-auto-sourcing`

> [!NOTE]
> If `rtw` command is not available, then add this to `~/.bashrc`:
> `export PATH=${PATH}:~/.local/bin`

> [!NOTE]
> The /tmp/.dockerdw5y4_1g.xauth can be differently named in your case.\
> Error response from daemon: failed to create task for container: failed to create shim task: OCI runtime create failed: runc create failed: unable to start container process: error during container init: error mounting "/tmp/.dockerdw5y4_1g.xauth" to rootfs at "/tmp/.dockerdw5y4_1g.xauth": mount /tmp/.dockerdw5y4_1g.xauth:/tmp/.dockerdw5y4_1g.xauth (via /proc/self/fd/7), flags: 0x5000: not a directory: unknown: Are you trying to mount a directory onto a file (or vice-versa)? Check if the specified host path exists and is the expected type
Error: failed to start containers: rtw_ctrlx_04_03_24_final-instance\
> check if /tmp/.dockerdw5y4_1g.xauth exists and is a file. If it is a directory remove first.\
> create the file: `sudo touch /tmp/.dockerdw5y4_1g.xauth`\
> `echo $(xauth nlist :0 | sed -e 's/^..../ffff/') | sudo xauth -f /tmp/.dockerdw5y4_1g.xauth nmerge -`

## porting workspace

add explicit export to variables: `bash ros_team_workspace/scripts/update-rtw.bash`

source new methods with exports: `source ~/.ros_team_ws_rc`

source your workspace: `_<ws_name>`

port config: `rtw wokspace port`

test porting: `rtw wokspace use`
