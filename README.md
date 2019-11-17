# path_optimization
分为两阶段:  
1. 对输入的路径进行平滑，目标函数里不考虑车辆约束和障碍物，速度快；  
2. 将平滑后的路径作为参考路径进行优化，用离散的线性化的车辆模型，考虑障碍物和起点终点位姿等，优化问题的形式是二次规划。  
## 图示
输入（灰色）：
![in.png](https://i.loli.net/2019/10/31/v8rGYNW6RHxOcwy.png)  
平滑：  
![smt.png](https://i.loli.net/2019/10/31/bTsxhRrW1LnJIoQ.png)  
第二次优化：    
![cargeo.png](https://i.loli.net/2019/10/31/dqz6TF4ypvYGbeE.png)  
## 依赖

- IPOPT    
- google [benckmark](https://github.com/google/benchmark)
- [osqp-eigen](https://github.com/robotology/osqp-eigen)  
  - [osqp](https://github.com/oxfordcontrol/osqp)    
  - [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)

## 运行  

### 测试 

克隆分支mpc_path_optimization_display到本地

```
git clone -b mpc_path_optimization_display git@github.com:bit-ivrc/path_optimizer.git
```

将文件夹重命名，比如display。  
克隆master分支到本地：  

```
git clone git@github.com:bit-ivrc/path_optimizer.git
```

将主分支的path_optimizer包复制到display/ivrc_ws/src/planning/dependent_package，进入工作空间编译运行state_sampling：

*注：编译过程如果遇到了//usr/lib/x86_64-linux-gnu/libapr-1.so.0: undefined reference to ‘uuid_generate@UUID_1.0'的问题，请参考[这篇博客](https://blog.csdn.net/u014734886/article/details/93029349)解决。感谢王威师兄提醒。*  

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
