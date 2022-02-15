# scenario_extraction_framework
This is a code base for the scenario extraction framework paper. There are two stages in code base: scenario extraction from real-world data and OpenX files generation. We have two docker containers for first and second stage. In the first docker container, ROS Melodic and all other dependencies to run the first stage is added. ROS melodic runs on python2, but second stage require python3. Once we have the sceanrio extractino result finished in the first stage, we use the second docker container for generating the OpenX files.

## Steps to setup the first docker container for extracting the sceanrio from the rosbag file
Before setting up the docker container, we have to edit the a few things as per the below steps:

