# pbr_gazebo
Precariously balanced rock gazebo simulation  
Zhiang Chen, zch@asu.edu

---
## Add Path
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/pbr_gazebo/models${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}$
```
## Download Models
1. [ros_vision](https://github.com/ZhiangChen/ros_vision)
2. [granite_dell.zip](https://download.openuas.us/granite_dell.zip)

## Simulation
### Simple Shake Table
[![Video](./doc/simple_shaking_table.png)](https://www.youtube.com/watch?v=8tYpVeeXM_s&t=10s)
```
roslaunch pbr_gazebo prismatic_box.launch
rosrun pbr_gazebo pulse_motion_server.py
rosrun pbr_gazebo pulse_motion_client.py 1 2
```
The pulse motion is deployed by a cosine function: <img src="https://render.githubusercontent.com/render/math?math=d = A-Acos(2\pi F t)">.  
There are two arguments required for pulse_motion_client.py. The first one is the amplitude of displacement, A; the second one is the fruquence of displacement, F. 

To reset shaking table displacement,
```
rosrun pbr_gazebo pulse_motion_client.py 0 0
```

`rqt` is recommended to plot the motion. There are three important topics:
```
/prismatic_box_controller/prismatic_joint_controller/command/data
/prismatic_box_controller/joint_states/velocity[0]
/prismatic_box_controller/joint_states/position[0]
```

To tune the PID parameters, [prismatic_box_controller.yaml](https://github.com/DREAMS-lab/pbr_gazebo/blob/master/config/prismatic_box_controller.yaml)


### Granite Dell
[![Video](./doc/granite_dell.png)](https://www.youtube.com/watch?v=9lwKEj10frs)
```
roslaunch pbr_gazebo granite_dell.launch
roslaunch pbr_gazebo prismatic_controller.launch
rosrun pbr_gazebo pulse_motion_server.py
rosrun pbr_gazebo pulse_motion_client.py 1 2
```
You might need to wait until `granite_dell` is fully launched in gazebo to launch the `prismatic_controller`.

To tune the PID parameters, [prismatic_controller_granite_dell.yaml](https://github.com/DREAMS-lab/pbr_gazebo/blob/master/config/prismatic_controller_granite_dell.yaml)

### PBRs and Boxes
PBR and box models can be loaded after the environment has been loaded in Gazebo
```
roslaunch pbr_gazebo add_pbr.launch
roslaunch pbr_gazebo add_box.launch
```
There are [box models with different dimensions](https://github.com/DREAMS-lab/pbr_gazebo/tree/master/models/rock_models). They can be specified in `add_box.launch` file to be loaded.


## PBR Parameters
1. coefficient of friction: 0.6  
2. terrain motion (DoF): 3 DoF  
3. amplitude and frequency: 
    - acceleration 1~1.5 g, 1g (+); 
    - single stroke;
    - displacement 1m;
    - velocity 2m/s; (-)
4. density: 2700 kg/m^3  
5. Elasticity (--)
