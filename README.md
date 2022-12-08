# opti_for_deformation

![Image](pictures/result.png?raw=true "result")

## Description
This code aimed to track feature points in the images by the method of optical flow. The green points are the tracked features, and the corresponding red numbers are their individual number to track.
## Prerequisites
1. OpenCV 

## Run The Data use_001_101
* comment out `#define dataset use_002_010` and `#define dataset use_012_020`, if there are other datasets.
![Image](pictures/tree_data.png?raw=true "tree_data")

* after building the codes, run it.
	* general mode: `rosrun deformation opti_flow`
	* getting the poixel positions of certain points like number 302, 392, 389: `rosrun deformation opti_flow 302 392 389`


