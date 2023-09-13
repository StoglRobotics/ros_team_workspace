# rtw cli installation

`git clone https://github.com/StoglRobotics/ros_team_workspace.git`

`cd ros_team_workspace/rtwcli/ && pip3 install -r requirements.txt`

`source ros_team_workspace/setup.bash`

`setup-auto-sourcing`

## porting workspace

add explicit export to variables: `bash ros_team_workspace/scripts/update-rtw.bash`

source new methods with exports: `source ~/.ros_team_ws_rc`

source your workspace: `_<ws_name>`

port config: `rtw wokspace port`

test porting: `rtw wokspace use`
