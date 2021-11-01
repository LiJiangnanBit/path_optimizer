# path_optimizer  
# :bell: CHECK OUT THE NEWER VERSION [path_optimizer_2](https://github.com/LiJiangnanBit/path_optimizer_2)  
![](https://img.shields.io/badge/ROS-Kinetic-orange)  ![](https://img.shields.io/github/last-commit/LijiangnanBit/path_optimizer)  
**This ROS package generates feasible paths for non-holonomic vehicles.**  
![2021-01-07 01-19-29屏幕截图.png](https://i.loli.net/2021/01/07/W9PHE2h5DUtsTi1.png)  

## Simulation videos
#### (1) [Simulation in dynamic environment](https://vimeo.com/498950818)  
#### (2) [Simulation with complex static obstacles](https://vimeo.com/498591477)  
![simulation](https://i.loli.net/2021/01/09/fkjmlpRXFxA9a2J.png)

## Run demos 
### 0. Install dependencies and build  
- ROS kinetic on Ubuntu 16.04 
- OpenCV 3    
~~~
mkdir -p workspace/src && cd workspace/src
git clone git@github.com:LiJiangnanBit/path_optimizer.git
sudo bash path_optimizer/scripts/install_deps.sh
cd ..
catkin build path_optimizer
source devel/setup.bash
~~~   
*install_deps.sh* will install other dependencies (Those already installed will be skipped).
These dependencies include:
- [ipopt 3.12.4](https://coding.net/u/aRagdoll/p/Ipopt-3.12.4/git)
- [cppad 20180000.0](https://www.coin-or.org/download/source/CppAD/cppad-20180000.0.gpl.tgz)
- [google benchmark](https://github.com/google/benchmark)
- [glog](https://github.com/google/glog)
- [gflags](https://github.com/gflags/gflags)
- [osqp-eigen](https://github.com/robotology/osqp-eigen)
- [grid_map](https://github.com/ANYbotics/grid_map)
- [ros_viz_tools](https://github.com/Magic-wei/ros_viz_tools)
- [tinyspline_ros](https://github.com/qutas/tinyspline_ros).  

### 1. Demo
A png image is loaded as the grid map. You can click to specify the global reference path and the start/goal state of the vehicle.  
~~~
roslaunch path_optimizer demo.launch
~~~
#### (1) Pick reference points using "Publish Point" tool in RViz.  
- Pick at least six points.  
- There are no hard and fast rules about the spacing of the points.  
- If you want to abandon the chosen points, just double click anywhere when using the "Publish Point" tool.  
- You can replace `gridmap.png` with other black and white images. Note that the resolution in `demo.cpp` is set to 0.2m, whick means that the length of one pixel is 0.2m on the map.  
- In application, the reference path is given by a global path or by a search algorithm like A*.  

![选点.gif](https://i.loli.net/2020/04/12/kRItwQTh5GJWHxV.gif)  
#### (2) Pick start state using "2D Pose Estimate" tool and pick goal state using "2D Nav Goal" tool.  
- Currently, it's not strictly required to reach the goal state. But this can be changed.    
- The start state must be ahead of the first reference point.  

![规划.gif](https://i.loli.net/2020/04/12/XmxgwTGRI1MtoVK.gif)  

### 2. Benchmark test  
This is a computation time test.

```
rosrun path_optimizer path_optimizer_benchmark
```   

## Usage
Refer to [demo.cpp](https://github.com/LiJiangnanBit/path_optimizer/blob/master/src/test/demo.cpp)  
The parameters that you can change can be found in `planning_flags.cpp`.  

## How it works
### Refer [here](https://github.com/LiJiangnanBit/path_optimizer/wiki).
1. Take inputs (red dots):  
![2020-02-13 16-27-46屏幕截图.png](https://i.loli.net/2020/02/13/rRdA7ZGmjfObzNV.png)  
2. (Optional) Use B spline curve fitting to make the path continuous and then search around it for a more reasonable reference path (yellow dots). 
This step can be skipped by changing settings.    
![2020-02-13 16-27-56屏幕截图.png](https://i.loli.net/2020/02/13/GJEbrUIXwScKmWT.png)    
3. Smooth the reference path using IPOPT (yellow curve).    
![2020-02-13 16-28-05屏幕截图.png](https://i.loli.net/2020/02/13/Meqi3m7CXzZFIxJ.png)  
4. Represent the path planning problem as a QP and solve it using OSQP.   
![2020-02-13 16-28-19屏幕截图.png](https://i.loli.net/2020/02/13/HaMpYKcZLxTdtAs.png)


