#!/bin/bash

if [ -z "$1" ]
  then
    echo "**************************************************************"
    echo "No first argument supplied, Select one of the below"
    echo "----------------------------------------------------"
    echo ""
    echo "start_first_container: Start second docker container"
    echo ""
    echo "start_second_container: Start second docker container"
    echo ""
    echo "remove_untagged: Remove the untagged images"
    echo ""
    echo "build_first_image: Build the first docker image"
    echo ""
    echo "build_first_imgage: Build the second docker image"
    echo ""
    echo "create_first_container: Build the first docker container"
    echo ""
    echo "create_second_container: Build the second docker container"
    echo ""
    echo "**************************************************************"
    echo ""
    echo "And pass second argument as well: 'full path of local folder of the
    project'"
    echo ""
    echo "Example: bash helper.bash build_first_image /home/Documents/scenario_extraction_framework"
    exit 1
fi

if [ -z "$2" ]
  then
    echo "No location argument supplied"
    exit 1
fi

first_docker_i=validation_first_i
first_docker_c=validation_first_c
second_docker_i=validation_second_i
second_docker_c=validation_second_c
home_volume="$2":/validation
#data_volume=/home/beastan/Documents/phd/dataset:/data
echo "Running '$1' and '$2' options"

# Start the first docker container
if [[ $1 == "start_first_container" ]]; then
  docker start $first_docker_c
	docker exec -it $first_docker_c /bin/bash
fi

# Start the second docker container
if [[ $1 == "start_second_container" ]]; then
  docker start $second_docker_c
	docker exec -it $second_docker_c /bin/bash
fi

# Remove untagged docker images
if [[ $1 == "remove_untagged" ]]; then
	docker rmi -f $(docker images --filter "dangling=true" -q --no-trunc)
fi

# Build first docker image
if [[ $1 == "build_first_image" ]]; then
  docker build -t $first_docker_i .
fi

# Build second docker image
if [[ $1 == "build_second_image" ]]; then
  docker build -t $second_docker_i .
fi

# create first docker container
if [[ $1 == "create_first_container" ]]; then
  		docker run --net=host -d -v $home_volume --name $first_docker_c $first_docker_i
fi

# create second docker container
if [[ $1 == "create_second_container" ]]; then
  		docker run --net=host -d -v $home_volume --name $second_docker_c $second_docker_i
fi




