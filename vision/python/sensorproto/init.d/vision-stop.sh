#!/bin/bash

#killall -v -r '.*sensor.*'
killall -v -u pi python
sleep 2
#killall -v -9 -r '.*sensor.*'
killall -v -9 -u pi python
