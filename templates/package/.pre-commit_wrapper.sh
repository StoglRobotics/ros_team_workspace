#!/bin/bash
source /opt/ros/foxy/setup.bash

cmd=$1
shift
$cmd "$@"
