# path_optimizer
**This is a ROS package for real-time path planning. It generates feasible paths for non-holonomic vehicles according to reference paths (discrete points).**
![2020-02-19 19-37-46屏幕截图.png](https://i.loli.net/2020/02/19/WHomBSMfyZ4jR62.png)  
## Run demos 
### dependencies 
- ROS kinetic on Ubuntu 16.04 
- IPOPT (version >=3.12.4) [Run this script](https://coding.net/u/aRagdoll/p/Ipopt-3.12.4/git/blob/master/install_ipopt.bash)   
- OpenCV  
- [google benckmark](https://github.com/google/benchmark)
- [osqp-eigen](https://github.com/robotology/osqp-eigen)  
  - [osqp](https://github.com/oxfordcontrol/osqp)    
  - [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
##### Other ROS packages  
- [grid_map](https://github.com/ANYbotics/grid_map)
- [ros_viz_tools](https://github.com/Magic-wei/ros_viz_tools)
- [tinyspline_ros](https://github.com/qutas/tinyspline_ros)  
  
### 1. Pick reference points manually
~~~
catkin build path_optimizer
roslaunch path_optimizer demo.launch
~~~
#### (1) Pick reference points using "Publish Point" tool in RViz.  
- Pick at least six points.  
- There are no hard and fast rules about the spacing of the points.  
- If you want to abandon the chosen points, just double click anywhere when using the "Publish Point" tool.  
- In application, the reference path is given by a global path or by a search algorithm like A*.  
![ref.gif](https://i.loli.net/2020/02/13/EXB8Qh9MdUOlm1R.gif)  
#### (2) Pick start state using "2D Pose Estimate" tool and pick goal state using "2D Nav Goal" tool.  
Currently, it's not strictly required to reach the goal state. But this can be changed.    
![calsulate.gif](https://i.loli.net/2020/02/13/mLxIkj4Kvirg7eO.gif)

### 2. Benchmark test  

```
rosrun path_optimizer path_optimizer_benchmark
```   
### 3. Simulation video
[![simulation](https://i.loli.net/2020/02/14/cIdRVs7GUhuTayv.png)](https://vimeo.com/391392050)

## How it works
1. Take inputs (red dots):
![2020-02-13 16-27-46屏幕截图.png](https://i.loli.net/2020/02/13/rRdA7ZGmjfObzNV.png)  
2. (Optional) Use B spline curve fitting to make the path continuous and then search around it for a more reasonable reference path (yellow dots). 
This step can be skipped by changing settings. 
![2020-02-13 16-27-56屏幕截图.png](https://i.loli.net/2020/02/13/GJEbrUIXwScKmWT.png)    
3. Smooth the reference path using IPOPT (yellow curve).   
![2020-02-13 16-28-05屏幕截图.png](https://i.loli.net/2020/02/13/Meqi3m7CXzZFIxJ.png)  
4. Represent the path planning problem as a QP and solve it using OSQP.  
![2020-02-13 16-28-19屏幕截图.png](https://i.loli.net/2020/02/13/HaMpYKcZLxTdtAs.png)
