#!/bin/sh

# 2013-04-04 Ben Adler:
# This script is meant to be called from a ros launch file in order to start MIRA.
# You should pass necessary arguments (like "-c PilotDemo.xml"). In order for this
# to work, we need to change into the path where these XML files are located.
# Also, roslaunch will add arguments (e.g. __name:=MIRA) to the arguments to this
# script. If we pass those to MIRA, it will fail. As a hack, we only pass the first
# two arguments $1 and $2, instead of passing all of them using $@

cd /localhome/demo/Domestic/etc/

echo "Starting MIRA framework (start_mira.sh): $1 $2"
/opt/MIRA/bin/mira $1 $2
