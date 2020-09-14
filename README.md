# pbr_gazebo
Precariously balanced rocks in gazebo simulation

---
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/pbr_gazebo/models${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}$
```

1. coefficient of friction: 0.6

2. terrain motion (DoF): 3 DoF 

3. amplitude and frequency: 
    acceleration 1~1.5 g, 1g (+); 
    single stroke;
    displacement 1m;
    velocity 2m/s; (-)
    
4. density: 2700 kg/m^3

5. Elasticity (--)