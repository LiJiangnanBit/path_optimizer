# mpc_path_optimization
A substitute of state sampling; path optimization using MPC.
## 未完成
- 现在只做了起点在路上的情况；
- 没有检查结果是否有碰撞；
- 对横向误差加约束，将结果限制在可通行区域内;
- 偶尔发生程序卡顿，原因暂时没有找到。
## 测试
切换到分支mpc_path_optimization_display
```
catkin build state_sampling
roslaunch state_sampling state_sampling.launch
```
蓝色为状态采样，黄色为MPC优化结果，红色为纯跟踪采样。
## 效果图
（蓝色为状态采样，黄色为使用MPC优化的结果）
![image](https://github.com/bit-ivrc/mpc_path_optimization/blob/master/picture/2019-08-23%2016-23-38%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)
![image](https://github.com/bit-ivrc/mpc_path_optimization/blob/master/picture/2019-08-23%2016-24-26%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)
![image](https://github.com/bit-ivrc/mpc_path_optimization/blob/master/picture/2019-08-23%2016-26-36%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)
![image](https://github.com/bit-ivrc/mpc_path_optimization/blob/master/picture/2019-08-23%2016-27-04%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)
![image](https://github.com/bit-ivrc/mpc_path_optimization/blob/master/picture/2019-08-23%2016-27-46%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)
![image](https://github.com/bit-ivrc/mpc_path_optimization/blob/master/picture/2019-08-23%2017-09-34%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)
