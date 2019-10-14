# path_optimization
## 依赖
IPOPT  
Benckmark  
Clothoids https://github.com/ebertolazzi/Clothoids
## 未完成内容
- [x] ~~检查结果是否存在碰撞~~  
- [x] ~~对横向误差加约束，将结果大概限制在可通行区域内~~  
- [ ] 输出的路径要考虑起始点的速度  
- [x] ~~点换成hmpl中的类型~~  
## 运行
### 编译
```
catkin build mpc_path_optimizer  
```
### Benchmark效率测试
```
rosrun mpc_path_optimizer mpc_path_optimizer_benchmark
```
### 显示对比
切换到分支mpc_path_optimization_display
```
catkin build state_sampling
roslaunch state_sampling state_sampling.launch
```
在rviz中使用工具栏上的2D Pose Estimate和2D Nav Goal作为规划起点和终点进行测试。黄色为路径优化结果，蓝色为状态采样。
