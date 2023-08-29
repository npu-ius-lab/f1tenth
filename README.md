# f1tenth

The second-place strategy in F1TENTH ICAUS 2022.

## Build
In catkin_ws/src:
```
git clone git@github.com:npu-iuslab/f1tenth.git
cd ..
catkin_make
```

## Usage
Start a simulator:
```
source devel/setup.bash
roslaunch f1tenth_simulator simulator.launch

```

Run!
```
source devel/setup.bash
rosrun f1tenth_simulator run_in_sim.py
```
