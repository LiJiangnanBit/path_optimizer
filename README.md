# path_optimization
## QP version  
分为两阶段:  
1. 对输入的路径进行平滑，目标函数里不考虑车辆约束和障碍物，速度快；  
2. 将平滑后的路径作为参考路径进行优化，用离散的线性化的车辆模型，考虑障碍物和起点终点位姿等，优化问题的形式是二次规划。  
## 图示
输入（灰色）：
![image](https://github.com/bit-ivrc/path_optimizer/blob/QP_version/picture/input.png)  
第一次优化：  
![image](https://github.com/bit-ivrc/path_optimizer/blob/QP_version/picture/smoothing.png)  
第二次优化：  
![image](https://github.com/bit-ivrc/path_optimizer/blob/QP_version/picture/optimization.png)  
结果：  
![image](https://github.com/bit-ivrc/path_optimizer/blob/QP_version/picture/optimization_geo.png)  
