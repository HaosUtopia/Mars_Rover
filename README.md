# Mars_Rover

![](https://github.com/HaosUtopia/Mars_Rover/blob/main/deeplabv3plus_ros/imgs/mars_env_gazebo.png)

visual servo for mars rover

### Start the simulation without mars environment
```bash
roslaunch curiosity_mars_rover_description main_simple.launch
```

### Start the simulation with mars environment

```bash
roslaunch curiosity_mars_rover_description main_real_mars.launch
```

### Start the controllers

```bash
roslaunch curiosity_mars_rover_controllers curiosity_mars_rover_controllers.launch

### Open the mast camera
rosservice call /curiosity_mars_rover/mast_controllers/set_mode 
mode_name: "open"
```

### Run object tracking demo

```bash
### Run simple environment
roslaunch curiosity_mars_rover_description main_simple.launch

### Run object tracking system
roslaunch deeplabv3plus_ros example.launch
```



