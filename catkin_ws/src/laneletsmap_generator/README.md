# Package: laneletsmap_generator

## Description:
Projects the ibeo lidar data from a bag file into an image and laneletmap in osm format.

### laneletsmap_generator

**Description** : Projects the ibeo lidar information into a 2d grid which is output to an image file and laneletmap in osm format

**Parameters**
* bag_file: bag file for input data
* output_image: output image file (can be any opencv type)
* laneletsmap_file: location to save laneletsmap

* min_intensity, max_intensity: range of lidar intensity for colourising the projection - different lidars have different ranges of reported intensity

* parameters for the h264 bag playback - some useful parameters listed below, for more parameters see h264_bag_playback project
  * percentage_start: place in the bag file to start
  * percentage_end: place in the bag file to end
  * horizon_in_buffer: set value="true" to enable the correction of the moving platform using the IMU

* use_rings: an array of numbers for the lidar ring numbers to use in the projection

**Author** : Dhanoop karunakaran
