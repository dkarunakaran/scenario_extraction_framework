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
```
cd /validation/parameters/
```

```
python3 generate_openx.py
```

```
python3 esmini_data.py
```

```
python3 plot.py
```
