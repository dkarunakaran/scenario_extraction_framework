#!/bin/bash

if [ -z "$1" ]
  then
    echo "**************************************************************"
    echo "No first argument supplied, Select one of the below"
    echo "----------------------------------------------------"
    echo ""
    echo "start_server: Pull and start the carla server docker" 
    echo ""
    echo "stop_server: Stop the carla server docker" 
    echo ""
    echo "start_client: Start the client docker"
    echo ""
    echo "resume_client: Resume the already started client docker"
    echo ""
    echo "stop_client: Stop the client docker"
    echo ""
    echo "remove_untagged: Remove the untagged images"
    echo ""
    echo "build_client_i: Build the client docker image"
    echo ""
    echo "build_client_c: Build the client docker container"
    echo ""
    echo "build_rviz_i: Build the rviz docker image"
    echo ""
    echo "run_rviz_c: Build and run the rviz docker container"
    echo ""
    echo "**************************************************************"
    echo ""
    echo "And pass second argument as well: 'laptop or monolith'"
    echo ""
    exit 1
fi

if [ -z "$2" ]
  then
    echo "No second argument supplied, pass 'laptop or monolith'"
    exit 1
fi

server_docker_c=carla-sim
client_docker_i=validation_i
client_docker_c=validation_c
rviz_docker_i=validation_rviz_i
rviz_docker_c=validation_rviz_c
home_volume=/home/beastan/Documents/phd/scenario_extraction:/model
data_volume=/home/beastan/Documents/phd/dataset:/data
echo "Running '$1' and '$2' options"

# Start the carla server
if [[ $1 == "start_server" ]]; then
	docker pull carlasim/carla:0.9.8
	if [[ $2 == "laptop" ]]; then
  		docker run --name=$server_docker_c -d=true --net=host --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=0 carlasim/carla:0.9.8 /bin/bash CarlaUE4.sh
	fi
	
	if [[ $2 == "monolith" ]]; then
  		docker run --name=$server_docker_c -d=true --net=host --gpus 2 carlasim/carla:0.9.8 /bin/bash CarlaUE4.sh
	fi
fi

# Stop the carla server
if [[ $1 == "stop_server" ]]; then
        docker stop $server_docker_c
        docker rm $server_docker_c
fi

# Start the client docker
if [[ $1 == "start_client" ]]; then
        docker start $client_docker_c
	docker exec -it $client_docker_c /bin/bash
fi

# Stop the client docker
if [[ $1 == "stop_client" ]]; then
        docker stop $client_docker_c
fi

# Resume the client docker
if [[ $1 == "resume_client" ]]; then
	docker exec -it $client_docker_c /bin/bash
fi

# Remove untagged docker images
if [[ $1 == "remove_untagged" ]]; then
	docker rmi -f $(docker images --filter "dangling=true" -q --no-trunc)
fi

# Build docker client image
if [[ $1 == "build_client_i" ]]; then
	cp -r ~/.ssh .
        	
	# Build the image
        docker build -t $client_docker_i .
        rm -rf .ssh
fi

# Build docker client container
if [[ $1 == "build_client_c" ]]; then
	if [[ $2 == "laptop" ]]; then
  		docker run --net=host -d -v $home_volume -v $data_volume --name $client_docker_c $client_docker_i
	fi

	if [[ $2 == "monolith" ]]; then
  		docker run --net=host -d -v $home_volume --name $client_docker_c $client_docker_i
	fi
fi

# Build Rviz docker image
if [[ $1 == "build_rviz_i" ]]; then
	cp -r ~/.ssh .

	# Build the image
	docker build -t $rviz_docker_i .
	rm -rf .ssh
fi

# Run Rviz docker container
if [[ $1 == "run_rviz_c" ]]; then
	xhost +local:docker
	if [[ $2 == "laptop" ]]; then
        	docker run -it --rm --privileged --net=host --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all --env=DISPLAY --env=QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v $home_volume --gpus 0 $rviz_docker_i /bin/bash

	fi

        if [[ $2 == "monolith" ]]; then
        	docker run -it --rm --privileged --net=host --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all --env=DISPLAY --env=QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix --gpus 0 $rviz_docker_i /bin/bash
	fi
fi



