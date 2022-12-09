# opti_for_deformation

![Image](pictures/result.png?raw=true "result")

## Description
This code aimed to track feature points in the images by the method of optical flow. The green points are the tracked features, and the corresponding red numbers are their individual number to track.

## Prerequisites
1. OpenCV 

## Parameters Needed to Be Set
* folder contain the images: `dataset = [folder_name]`
* image amount to be tracked: `img_amount = [amount of images]`
* output path to save results: `save_path = [path to save]`

## Build and Run
* Build: ``g++ -o optical_flow optical_flow.cpp `pkg-config --cflags --libs opencv```
* Run: `./optical_flow`

## Region of Interist
* Focus the feature points in ROI.
`./optical_flow [lower_u] [upper_u] [lower_v] [upper_v]`

exp:
* Focus u_pixel from 0 ~ 700 and v_pixel from 0 ~ 514.
`./optical_flow 0 700 0 514`

## Results
* The first column is the number of feature point.
* Row的話依序是第一個pt的x跟y座標, 第二個pt的x 跟y座標, 第三, 第四…. Column的話, 則是這些個points在不同frame 上的位置.


