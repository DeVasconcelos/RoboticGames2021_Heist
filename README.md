# Robotic Games 2021 - Heist

This repository houses our implementation of the evader for the final project of the Robotic Games 2021 lecture. The code for the game itself, can be found here: https://github.com/holger-ziti/RG2021_projects, and needs to be cloned to your catkin workspace beforehand.

## How to run the project 
To start the first map, run:

```bash
$ roslaunch heist map_1.launch
$ roslaunch evader evader_map1.launch
```

To start the second map, run:
```bash
$ roslaunch heist map_2.launch
$ roslaunch evader evader_map2.launch
```

## Known issues with the Rogata Engine

To decrease the runtime of our implementation, comment out the following line in src/rogata-library/rogata_library.py of the Rogata Engine (line 392):
```python
#(rospy.wait_for_service('intersect_line',0.5)) 
```
