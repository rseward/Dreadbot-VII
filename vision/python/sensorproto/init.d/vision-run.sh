#!/bin/bash

# Copy this to /home/pi to facilate the easier starting of the vision component

# Start the sensor. Also note you can pass --debug manually to the python script
#  with an HDMI monitor connected to view what opencv sees on the camera.
# ./start-vision.sh --debug
# or ./sensor.py --debug when you are in the sensorproto directory.
$HOME/rseward/bitbucket/dreadbots/sensorproto/sensor.py $*

