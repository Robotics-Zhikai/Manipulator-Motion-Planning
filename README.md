# Manipulator-Motion-Planning
四自由度机械臂运动规划，应用背景是挖掘机挖土从A到B

## 20200707
解决直线与曲面相交的碰撞存在性判断，一定程度上复现了vpasolve函数，该方法的精度比暴力的密集撒点高，且更快
## 20200709 
单纯用带参数的高次多项式带约束进行规划是比较复杂的，可能涉及到凸优化的知识。替代性的，用固有的单调曲线进行规划，加速度连续且速度连续。
## 20200710
关节空间下点到点的保证加速度和速度连续的路径规划，用到的方法是中间段线性，靠近初末位置有连续的加速度存在。积分式很复杂，需要MATLAB进行运算。
## 20200713 
考虑挖斗的运动规划，挖斗需要做运动平滑
## 20200714
挖斗在挖土的过程中保持直线，用雅可比矩阵去做约束 
## 20200716
一般空间直线考虑关节空间速度和加速度约束的启发式算法
## 20200717 
修复相关bug 生成更好的轨迹曲线
## 20200718-0721
构建pid结构，更精确的规划到目标位置；提升算法鲁棒性
## 20200722
MATLAB代码生成 考虑铲斗保持内容物不漏的约束的关节空间轨迹规划初步研究
## 20200723 
封装铲斗与地面夹角范围和theta4的范围的对应关系函数
## 20200724
解决部分直线挖掘的bug
将所有运动学参数参数化，以便为以后更普遍的挖机的运动规划奠定基础
## 20200727 0728
算法参数化转换
## 20200729
参数化算法 借助凸包算法参数化可达性判断算法
## 20200730 731 
可视化整个臂 着手铲斗末端的考虑约束的直线规划
## 20200803
进一步完善机械臂的可视化程序 编写部分铲斗尖直线规划算法程序 将计算出来的雅可比矩阵编入程序
