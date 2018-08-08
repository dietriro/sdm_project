# Project: Sequential Decision Making in Robotics - Mobile Robot Path Planning using Monocular Vision based Localization

## Description

Using only monocular vision information for mobile robot localization is efficient, yet poses challenges due to the sensibility of the sensor. Sudden and quick turns of the robot can for example lead to motion blurr in the images and decrease the robots localization. In our project we therefore worked on a path planning algorithm based on ARC-Theta* which tries to minimize the angular difference between each two consecutive points in a path. This leads to an overall minimization of turns the robot has to take, since the planner prefers longer routes with smooth paths instead of shorter ones with many turns.
