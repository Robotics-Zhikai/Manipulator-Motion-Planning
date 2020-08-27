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
## 20200805
计算的雅可比矩阵似乎有错误 
## 20200806
计算的雅可比矩阵没有错误，而是雅可比矩阵提供的信息不充分，秩为2 着手新的算法演技
## 20200807
秩为2的原因是没有考虑铲斗的旋转，当在雅可比矩阵中加上铲斗的旋转项时，秩为3，可以求解。现在的问题是，如何对omigay进行范围的限定，使得表征铲尺直线运动速度的量k在各关节速度和加速度约束下有解.
## 20200810 0811
需要用计算几何的凸包交集算法进行速度加速度约束下的omiga和k可行区域求解。
## 20200813
编写凸包求交集的程序，有一些bug
## 20200814
解决bug，得到任意时刻可选的omigay 和 k 区域一般是平行四边形
## 20200817
omigay 和 k的选取是耦合的，如何解耦？？？
## 20200818
似乎无法解耦，这是一个序列的优化问题，适合人去做，可以考虑作为辅助人去直线挖掘的程序，满足速度与加速度约束 presorting的bug：去除LTL时的10-19
## 20200819
用robotics tool工具箱插值的方法非常快 把直线离散化求逆解 二分法 四元数 末端速度先增后平最后减
不需要用辅助人工 只不过离散化不是严格的直线 但离散化足够的情况下 是比之前做的工作好的。
## 20200820
可视化 修复一些bug 尝试转C语言 MATLAB code generation 似乎有点坑啊。。。
## 20200821
ctraj函数转化到c语言 用矩阵库Eigen就有MATLAB编程的感觉了
## 20200824 
转换cpp
## 20200825
编写内容物不漏的程序 验证cpp程序的正确性 发现似乎随时间误差是不一样的。。。
## 20200826
不在关节空间写内容物不漏的程序 在笛卡尔空间写 直上直到
## 20200827
基本完成先保持在释放内容物的程序 笛卡尔空间，直上直到 缺点是每一个中间点的速度都为0，看起来一顿一顿的 需要改进