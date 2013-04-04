#!/bin/sh

# 2013-04-04 Ben Adler:
# This script is meant to be called from a ros launch file in order to start MIRA.
# You should pass necessary arguments (like "-c PilotDemo.xml"). In order for this
# to work, we need to change into the path where these XML files are located.

cd /localhome/demo/Domestic/etc/
/opt/MIRA/bin/mira $@
