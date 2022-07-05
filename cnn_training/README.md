A dataset captured on the Císařský island in Prague in 2021 with an Ouster OS0-128 sensor.
This dataset is designated for training convolutional neural networks for the detection of drones from LiDAR images.
The rosbag contains the following topics of interest:

* `/lidar/sensor_info` - information about the LiDAR sensor used to capture the data
* `/lidar/lidar_packets` - raw packets from the Ouster sensor, contains lossless data captured by the LiDAR
* `/lidar/ambient_image` - 8-bit grayscale images representing the *ambient* channel captured by the LiDAR
* `/lidar/intensity_image` - 8-bit grayscale images representing the *intensity* channel captured by the LiDAR
* `/lidar/range_image` - 8-bit grayscale images representing the *range* channel captured by the LiDAR
* `/rgb/image_raw/compressed` - RGB images from the onboard camera
* `/labels` - bounding boxe and range of the target in the LiDAR images
* `/tf_static` - contains the transformation between the LiDAR frames and the RGB camera

