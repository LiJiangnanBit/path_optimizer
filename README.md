# path_optimizer
**This is a ROS package for real-time path planning. It generates feasible paths for a non holonomic vehicle according to reference paths (discrete points).**
![cover.png](https://i.loli.net/2020/02/11/CiZXwjQeGNaqdsr.png)  

## Run demos 
### dependencies 
- ROS kinetic on Ubuntu 16.04 
- [grid_map](https://github.com/ANYbotics/grid_map)
- IPOPT (version >=3.12.4)  
- OpenCV   
- [google benckmark](https://github.com/google/benchmark)
- [osqp-eigen](https://github.com/robotology/osqp-eigen)  
  - [osqp](https://github.com/oxfordcontrol/osqp)    
  - [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [ros_viz_tools](https://github.com/Magic-wei/ros_viz_tools)
  
### 1. Choose reference points manually
~~~
catkin build path_optimizer
roslaunch path_optimizer demo.launch
~~~
#### (1) Choose reference points using "Publish Point" tool in RViz.  
- Choose at least six points.  
- There are no hard and fast rules about the spacing of the points.  
- If you want to abandon the chosen points, just double click anywhere when using the "Publish Point" tool.  
- In application, the reference path is given by a global path or by a search algorithm like A*.  
![image](https://github.com/bit-ivrc/path_optimizer/blob/visualization/picture/ref.gif)  
#### Choose start state using "2D Pose Estimate" tool and choose goal state using "2D Nav Goal" tool.  
Currently, it's not strictly required to reach the goal state. But this can be changed.    
![image](https://github.com/bit-ivrc/path_optimizer/blob/visualization/picture/calsulate.gif)

### 2. Benchmark test  

```
rosrun path_optimizer path_optimizer_benchmark
``` 
## How it works
1. Take inputs (red dots):
![2020-02-13 16-27-46屏幕截图.png](https://i.loli.net/2020/02/13/rRdA7ZGmjfObzNV.png)  
2. Use B spline curve fitting to make the path continuous and then search around path for a more reasonable reference path. (yellow dots) 
This step can be skipped by change settings. 
![2020-02-13 16-27-56屏幕截图.png](https://i.loli.net/2020/02/13/GJEbrUIXwScKmWT.png)    
3. Smooth the reference path using IPOPT (yellow curve).   
![2020-02-13 16-28-05屏幕截图.png](https://i.loli.net/2020/02/13/Meqi3m7CXzZFIxJ.png)  
4. Represent the path planning problem as a QP and solve it using OSQP.  
![2020-02-13 16-28-19屏幕截图.png](https://i.loli.net/2020/02/13/HaMpYKcZLxTdtAs.png)
