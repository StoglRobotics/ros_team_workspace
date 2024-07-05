# rtw cli installation

`git clone https://github.com/StoglRobotics/ros_team_workspace.git`

`cd ros_team_workspace/rtwcli/ && pip3 install -r requirements.txt`

`source ros_team_workspace/setup.bash`

`setup-auto-sourcing`

> [!NOTE]
> If `rtw` command is not available, then add this to `~/.bashrc`:
> `export PATH=${PATH}:${HOME}/.local/bin`

## porting workspace

add explicit export to variables: `bash ros_team_workspace/scripts/update-rtw.bash`

source new methods with exports: `source ~/.ros_team_ws_rc`

source your workspace: `_<ws_name>`

port config: `rtw wokspace port`

test porting: `rtw wokspace use`


## For converting https to ssh:
> [!NOTE]
> `git config --global url."git@github.com:".insteadOf "https://github.com/"`
