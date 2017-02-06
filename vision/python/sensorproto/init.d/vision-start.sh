#!/bin/bash

# Copy this to /home/pi to facilate the easier starting of the vision component

# Start the sensor. Also note you can pass --debug manually to the python script
#  with an HDMI monitor connected to view what opencv sees on the camera.
# ./start-vision.sh --debug
# or ./sensor.py --debug when you are in the sensorproto directory.

scriptdir=$( dirname $0 )
screen -A -m -d -S vision $scriptdir/vision-run.sh
screen -ls

echo "Attach to the screen to see the output or run $scriptdir/vision-run.sh directly"
echo "screen -dR vision"

#$HOME/rseward/bitbucket/dreadbots/sensorproto/sensor.py $*

