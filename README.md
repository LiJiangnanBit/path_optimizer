# path_optimization
![image](https://github.com/bit-ivrc/path_optimizer/blob/master/picture/car_geo_result.png)  
![image](https://github.com/bit-ivrc/path_optimizer/blob/master/picture/contrast.png)  
## 依赖
- IPOPT    
- google [benckmark](https://github.com/google/benchmark)   
## 运行  
### 测试 
克隆分支mpc_path_optimization_display到本地
```
git clone -b mpc_path_optimization_display git@github.com:bit-ivrc/path_optimizer.git
```
将文件夹重命名，比如display。  
克隆主分支到本地：  
```
git clone git@github.com:bit-ivrc/path_optimizer.git
``` 
将主分支的path_optimizer包复制到display/ivrc_ws/src/planning/dependent_package，进入工作空间编译运行state_sampling：
```
cd display/ivrc_ws
catkin build state_sampling
roslaunch state_sampling state_sampling.launch
```
在rviz中使用工具栏上的2D Pose Estimate和2D Nav Goal作为规划起点和终点进行测试。黄色为路径优化结果，蓝色为状态采样。
### Benchmark效率测试  
```
rosrun path_optimizer path_optimizer_benchmark
```
