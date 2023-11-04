# f1tenth

The second-place strategy in F1TENTH ICAUS 2022. And This repository primarily focuses on the simulations in  F1TENTH environment, specifically based on reactive and navigation methods.

## Build
In catkin_ws/src:
```
git clone git@github.com:npu-iuslab/f1tenth.git
cd ..
catkin_make
source devel/setup.bash
```

## Usage
#### 1.Reactive control in  F1TENTH simulation environment
Start the simulator:
```
roslaunch f1tenth_reactive_simulator simulator.launch
```

Run the ROS control node:
```
rosrun f1tenth_reactive_simulator run_in_sim.py
```
#### 2.Navigation in  F1TENTH simulation environment
Start the simulator:
```
roslaunch f1tenth_tianracer_gazebo demo_tianracer_teb_nav.launch
```

Run the multi-point navigation control script:
```
rosrun f1tenth_tianracer_navigation tianracer_multi_goals.launch
```

## TroubleShooting
If you cannot start the relevant nodes, it is possible that you need to give execution permissions to Python files in the corresponding "scripts" folder.
