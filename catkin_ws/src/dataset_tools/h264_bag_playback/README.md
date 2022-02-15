**NOTE: this branch reads mp4 files, and allows the playback tool to seek immediately to the correct location in the file**

Usage: First, you need to convert the existing .h264 files into .mp4 files. There is a script dataset_tools/h264_bag_playback/scripts/convert_videos.sh

Run this script with ./convert_videos /root_folder_for_datasets
This script currently expects a folder containing one or more dataset folders (i.e. /data/Week1, /data/Week2, etc). The script is designed to work off the /data folder

Once the files are converted, run the playback tool the same way as before. The only difference you will notice is that the video seeks straight away, there is no "discarding frames" messages anymore.

Previous README:
h264_bag_tools is a c++ program that takes a bag file + h.264 files and replays them as if on a live system. The bag
file must contain frame_info_msg messages that link each frame of the h.264 video files to the bag. The playback
tool publishes topics for the uncorrected images /gmsl/\<camera name\>/image_color and the corrected images (given the
camera_info_msgs) on /gmsl/\<camera name\>/image_rect_color.

NOTE: The images are only extracted from the h.264 file if someone has subscribed to the topic. Because the h.264
encoding requires a sequence of frames, all of the images to the current time must be read in sequence. This means that
if you subscribe to an image topic after playback has commenced, the playback will pause while the program reads all of
the images up to the current time. This can be a bit slow if you subscribe to the image topic a long time after playback
starts.

parameters:

    name="output_width" value in pixels
    set the width of the images
    NOTE: if either of the width/height parameters are not defined (or set to 0) the
    original image size is used

    name="output_height" value in pixels
    set the height of the images
    NOTE: if either of the width/height parameters are not defined (or set to 0) the
    original image size is used

    <Optional>
    name="time_start" value in iso format (i.e. "2018-07-05T05:39:22.701861")
    If both of time_start and percentage_start is missing, it will play from the beginning
    On startup, the ROS INFO message will print the start and end times for the bag

    <Optional>
    name="time_end" value in iso format (i.e. "2018-07-05T05:40:22.701861")
    If both of time_end and playback_end is missing, it will play through to the end
    On startup, the ROS INFO message will print the start and end times for the bag

    <Optional>
    name="percentage_start" value in floating-point percentage (i.e. 23.4 will play from 23.4% of bag)
    If both of time_start and percentage_start is missing, it will play from the beginning
    If at least one of time_start and time_end parameter pair is specified, percentage_start and percentage_end won't be processed

    <Optional>
    name="percentage_end" value in floating-point percentage (i.e. 99.99 will play until 99.99% of bag)
    If the end is missing, it will play through to the end
    If at least one of time_start and time_end parameter pair is specified, percentage_start and percentage_end won't be processed

    <Optional>
    name="horizon_in_buffer" value in boolean, default to false
    if horizon_in_buffer set to true, will read in /vn100/imu messages and compute base_link->base_link_horizon and
    base_footprint->base_footprint_horizon. Buffer up to 2 seconds of horizon tfs in the future

    name="limit_playback_speed" value is boolean
    Either playback the bag + images as fast as possible, or restrict to (close to) 
    realtime playback

    name="bag_file" value is a string
    The name of the bag file to read. The h.264 files and camera names are 
    automatically extracted from the bag file name using the first part of the bag 
    file name (without the extension) and the camera name separated using an '_'. 
    Note that this is the standard format for the files used in the ACFR campus dataset

additional requirements:
* The gmsl_frame_msg package is required, and is available at https://gitlab.acfr.usyd.edu.au/nvidia/gmsl_frame_msg
* ros-kinetic-tf2-sensor-msgs or ros-melodic-tf2-sensor-msgs is required, this package is not usually installed by default in ROS and is available through apt

example usage:

    roslaunch h264_bag_playback h264_playback.launch bag_file_name:="/home/stew/data/callan-park/2019-04-15-14-37-06_callan_park_loop.bag"

    NOTE: change the output image size in h264_playback.launch



tests:

    a new roscore has to be spinned up for all tests to pass.

    catkin run_tests --no-deps --this --verbose
