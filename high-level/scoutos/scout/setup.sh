#!/bin/bash

cmake messages
cmake motors
cmake encoders
cmake power
cmake libscout
cd libscout
python generate_behavior_lists.py
cd ..
cmake scoutsim
