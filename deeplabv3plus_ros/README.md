# deeplabv3plus_ros
![](https://github.com/HaosUtopia/Mars_Rover/blob/main/deeplabv3plus_ros/imgs/mars_rover_mastcam_rock_tracking.gif)
This is a ROS package of [DeeplabV3+](https://arxiv.org/abs/1802.02611). Most of core algorighm code was based on [Deeplab implementation by tensorflow](https://github.com/tensorflow/models/tree/master/research/deeplab).

## Requirements

- ROS kinetic
- TensorFlow >= 1.14.0
- CUDA >= 10.0
- cuDNN >= 7.4.1

For more information, please visit this [link](https://github.com/tensorflow/models/blob/master/research/deeplab/g3doc/installation.md).

## Training

This repositry doesn't contain the code for training. If you want to train the model on your class definition or dataset, please refer to this [link](https://www.programmersought.com/article/4188126074/).

## Getting Started

Clone this repository to the src folder of your catkin workspace, build your workspace and source it.

```bash
   cd <catkin_ws>/src
   git clone https://github.com/Masahiro-Obuchi/deeplabv3plus_ros.git
   cd deeplabv3plus_ros/nodes
   chmod +x deeplabv3plus_ros.py
   cd ../../../
   catkin_make
   source <catkin_ws>/devel/setup.bash
```

## Example Usage

```bash
   roslaunch deeplabv3plus_ros example.launch
```

## ROS Node

### Parameters:

- `~model_path`[string]  
  The path to the model's frozen graph

### Topics Published:

- `seg_result: sensor_mgs/Image`  
  Visualized result over an input image
- `seg_map: sensor_msgs/Image`  
  Segmentation result

### Topics Subscribed

- `image: sensor_msgs/Image`  
  Input RGB image
