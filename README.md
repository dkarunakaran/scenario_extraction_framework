# scenario_extraction_framework
This is a code base for the scenario extraction framework paper. There are two stages in code base: scenario extraction from real-world data and OpenX files generation. We have two docker containers for first and second stage. In the first docker container, ROS Melodic and all other dependencies to run the first stage is added. ROS melodic runs on python2, but second stage require python3. Once we have the scenario extraction result finished in the first stage, we use the second docker container for generating the OpenX files.

## Steps to setup the first docker container for extracting the scenario from the rosbag file
* Build the first docker image
```
bash helper.bash build_first_image <full path of local project folder>
```
Example:
```
bash helper.bash build_first_image /home/something/Documents/scenario_extraction_framework
```

* Create the first container
```
 bash helper.bash create_first_container <full path of local project folder>
```

* Run the container
```
 bash helper.bash start_first_container <full path of local project folder>
```

* Once you are in container, naviagate to /validation/ and run the following code
```
bash lz4_streamdecode_error_fix.bash 
```

* Then change dir to catkin_ws, and run following code to build the ROS packages
```
catkin  build
```
If we have all 12 packages got installed and then good to go for next step to extract the scenarios.


## Run roslaunch file to extract scenarios
* Navigate to /validation/catkin_ws
```
cd /validation/catkin_ws
```

* Run roslaunch to extract scenario paramters from rosbag file

```
roslaunch launch/validation_dataset.launch bag_file:=<location of the bag file> output_image:=/validation/top_view_image_of_lanes.png laneletsmap_file:=/validation/map.osm
```

* If everythin goes smooth,  files with extension 'cutin' and 'cutout' have been created in the /validation/parameters/scenario_data/


## Steps to setup the second docker container for generating OpenX files
* Build the first docker image
```
bash helper.bash build_second_image <full path of local project folder>
```
Example:
```
bash helper.bash build_second_image /home/something/Documents/scenario_extraction_framework
```

* Create the first container
```
 bash helper.bash create_second_container <full path of local project folder>
```

* Run the container
```
 bash helper.bash start_second_container <full path of local project folder>
```



## Generate OpenX files
Once we are in the container, follow below steps

* Navigate the the parameters folder
```
cd /validation/parameters/
```

* Generate OpenX files such as OpenSCENARIO and OpendDRIVE files

```
python3 generate_openx.py
```

* This step will execute OpenX files in Esmini OpenSCENARIO player and generate date for plotting. At the moment we are running esmini headless as we haven't added GUI to docker.

```
python3 esmini_data.py
```

* Create some plots for visulaisation and you can find them in /validation/parameters/plots folder

```
python3 plot.py
```
