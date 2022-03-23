# HallerSim

### First run
```
  cd <package folder>
  catkin_make
```

### Running ROS Server
HallerSim ROS package serves as connector between ROS and Unity. It will collect messages from Unity and ROS and relay them between these two enviroments.
This package runs a server with host IP on port 10000.

Before running you have to set correct server host IP. Just go to **<package folder>/src/haller_sim/auvConfig/auvConfig.json** and set **rosIP** value to host machine IP.
You can get host IP by running:
```
  hostname -I
```
It typically has value **198.162.0.xxx**
  
Finally to run server endpoint:  
```
  cd <package folder>
  source devel/setup.bash
  roslaunch haller_sim endpoint.launch
```
### Running Unity Simulation

Go to folder containing downloaded build and unpack.
Then go to 
  
### BUILDS AVAILABLE AT

https://drive.google.com/drive/folders/1_mSJqCyk6Kb4jB5M2lmCJClVihBUw8Jm?usp=sharing
