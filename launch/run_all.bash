#!/bin/bash

roslaunch bw_generator bw_generator_1.launch &
sleep 2
roslaunch bw_generator bw_generator_2.launch &
sleep 2
roslaunch bw_generator bw_generator_3.launch &
sleep 2
roslaunch bw_generator bw_generator_4.launch &
sleep 2
roslaunch bw_generator bw_generator_5.launch &
sleep 2
roslaunch bw_generator bw_generator_6.launch &
sleep 2
roslaunch bw_generator bw_generator_7.launch &
sleep 2
roslaunch bw_generator bw_generator_8.launch &

