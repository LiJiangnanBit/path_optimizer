# path_optimizer
**Real-time path planning for non-holonomic vehicles.**
![cover.png](https://i.loli.net/2020/02/11/CiZXwjQeGNaqdsr.png)  


Tested on Ubuntu 16.04.  
## Installation
### Install dependencies
~~~
git clone git@github.com:LiJiangnanBit/path_optimizer.git
cd path_optimizer
sudo bash scripts/install_dependencies.sh
~~~
These dependencies include eigen, osqp, ipopt, google benchmark, [osqp-eigen](https://github.com/robotology/osqp-eigen) and [grid_map](https://github.com/ANYbotics/grid_map).   
### Build and install
~~~
mkdir build
cd build
cmake ..
make
sudo make install
~~~  
## Demos
### 1. [Run demo on ROS](https://github.com/LiJiangnanBit/path_optimizer_ROS_demo), pick reference points manually
![ref.gif](https://i.loli.net/2020/02/13/EXB8Qh9MdUOlm1R.gif)  
![calsulate.gif](https://i.loli.net/2020/02/13/mLxIkj4Kvirg7eO.gif)  
### 2. Simulation video
[![simulation](https://i.loli.net/2020/02/14/cIdRVs7GUhuTayv.png)](https://vimeo.com/391392050)
### 3. Benchmark test  

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
