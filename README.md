# mpc_path_optimization
### A substitute for state sampling; path optimization using MPC.
为了解决路径规划中的状态采样有时结果不太自然的问题(S形晃动，过大的偏差等)，改善路径质量，并提高成功率，尝试改用MPC的方法对搜索结果进行优化。
## 依赖
IPOPT  
Benckmark
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
在rviz中使用工具栏上的2D Pose Estimate和2D Nav Goal作为规划起点和终点进行测试。黄色为MPC优化结果，蓝色为状态采样，红色为纯跟踪采样。
## 效果图
（黄色为使用MPC优化的结果，蓝色为状态采样）
![image](https://github.com/bit-ivrc/mpc_path_optimization/blob/master/picture/2019-08-23%2016-23-38%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)
![image](https://github.com/bit-ivrc/mpc_path_optimization/blob/master/picture/2019-08-23%2016-24-26%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)
![image](https://github.com/bit-ivrc/mpc_path_optimization/blob/master/picture/2019-08-23%2016-26-36%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)
![image](https://github.com/bit-ivrc/mpc_path_optimization/blob/master/picture/2019-08-23%2016-27-04%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)
![image](https://github.com/bit-ivrc/mpc_path_optimization/blob/master/picture/2019-08-23%2016-27-46%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)
![image](https://github.com/bit-ivrc/mpc_path_optimization/blob/master/picture/2019-08-23%2017-09-34%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)
