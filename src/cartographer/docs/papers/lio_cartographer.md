
# 1 引言

​	随着人工智能技术的快速迭代和发展，机器人技术得以重大突破，尤其是巡检机器人和服务型机器人得到工业界和产业界越来越多的重视。在机器人行业中，SLAM(Simultaneous Localization and Mapping, 同步定位与建图)技术往往作为机器人中的核心模块，完成定位、建图等功能。在SLAM方向中，以传感器作为区分别，主要分为激光SLAM和视觉SLAM，在机器人工业化中运用中，激光由于其稳定性被大多数场景运用，其中激光SLAM可分为2D激光SLAM和3D激光SLAM，3D激光SLAM往往运用于室外复杂或较大场景，而在服务机器人行业中，单线激光雷达具有结构简单、成本低等特点，2D激光SLAM被广泛使用[1]。在2D激光SLAM中，被广泛接受的的分为基于图优化算法和基于滤波算法两种实现形式，其中基于图优化方案中，经典算法包括Cartographer算法[2]、karto算法[3]等，基于滤波的方案中经典算法包括gmapping[4]算法等，主要传感器包括单线激光雷达、IMU(Inertial Measurement Unit, 惯性测量单元)、轮速计等。

​	针对工业界和学术界，国内外学者提出多种方案和优化算法使2D激光SLAM能够在服务机器人中广泛运用。文献[5]提出一种基于位姿增量的多传感器位姿融合的Cartographer算法，主要针对无损卡尔曼滤波器计算量大和雷达观测时间与里程计存在时间延迟问题进行优化。在Cartographer算法中，主要使用CSM(Correlative Scan Matching)[6]算法进行粗匹配，使用多次概率栅格地图插值的形式对CSM结果进行细化，获取更加精细的配准结果，文献[7]通过对点云数据进行特征提取和识别提升位姿估计的初始精度，并通过匹配协方差进一步优化在退化场景的效果。文献[8]通过使用IMU预积分因子和激光匹配因子构成因子图优化的形式改进Cartographer算法。文献[9]提出CAE-RLSM算法用于构建在线特征图，其中主要以为线特征为主，该算法的目的即在于获取全局一致性特征地图，并未将该地图使用到SLAM算法模块中。文献[10]中提出SoMaSLAM算法，将曼哈顿世界(Manhattan World, MV)假设运用到2D 激光SLAM中，主要考虑到在人造环境中，大多出特征呈现直角特性，将该特性在图优化构建软约束。在上述算法中，主要通过增强线性特征提高匹配精度，从而提高SLAM前端的精度，然而在进行线段拟合，本身已经丧失一部分精度。在算法[11]中提出一种在复杂环境下基于改进图优化的二维激光SLAM算法，在该算法前端中，使用似然场匹配的形式增强配准精度，但是依然需要使用CSM匹配算法提供测配准位姿。另外在3D激光SLAM中，较多研究集中在使用滤波形式耦合IMU数据和激光雷达匹配位姿，如Fast-LIO算法[12]、Faster-LIO算法[13]等。基于此，本文提出一种基于高斯平滑的似然场融合二维SLAM算法：

​	使用IMU和激光雷达紧耦合形式构建位姿递推器，将二维似然场约束与IMU积分约束集成到同一框架中；

​	似然场地图由概率地图生成，在占据和非占据边缘处概率变化较大，往往呈现阶梯式变化，以图像存储的概率地图为例，概率变化往往存在于少量像素距离之类，在进行优化求解时，初始位姿差距过大，会因丢失梯度导致无法收敛，基于此，本文提出一种融合算法，即对似然场地图进行高斯平滑，提供粗配准，使用原始似然场地图进行精配准；

​	整个SLAM系统，以关键帧为最小单元，多个关键帧作为子地图构成元素，使用子地图进行回环检索，以图优化形式构成后端优化模块。

# 2 算法流程



# 3 算法框架

## 3.1 ESKF位姿递推器

使用$\tilde\omega$表示IMU角速度测量值，$a$表示IMU加速度测量值，使用$\eta_g$，$\eta_a$分别表示陀螺仪测量噪声和加速度计测量噪声，使用$b_g$，$b_a$分别表示陀螺仪和加速度计零偏。名义状态量表示为$X=[p,v,\theta,b_a,b_g]$，其中$\theta$表示为偏航角度，亦可使用$R$表示在IMU前向IMU递推中，可表示为：
$$
R(t+\Delta{t})=R(t)SO2((\tilde{w}-b_g)\Delta{t} \tag{式1}
$$

$$
p(t+\Delta{t})=p(t)+v\Delta{t}+\frac{1}{2}R(t)(\tilde{a}-b_a)\Delta{t}^2 \tag{式2}
$$

$$
v(t+\Delta{t})=v(t)+R(t)(\tilde{a}-b_a)\Delta{t} \tag{式3}
$$

其中$SO2(\omega)$表示旋转角转旋转矩阵。误差状态表示为$\delta{X}=[\delta{p},\delta{v} ,\delta{\theta}, \delta{b_a}, \delta{b_g}]$，



## 3.2似然场生成和配准

​	似然场的概率可描述为  $ p(z_t{\mid}x_t,m) $，$x_t{\in}SE2$表示为机器人状态，$m$表示为地图。激光雷达传感器获取点云使用$P_{l}=\{p_1,p_2,...,p_k\}$表示，激光雷达到机器人本体坐标系的外参表示为$T_{bl}$，车辆在地图坐标系的位姿表示为$T_{wb}$，则激光测量点在地图坐标系可表示为：
$$
P_m=T_{wb}T_{bl}P_l \tag{式1}
$$
​	对于任一帧激光点云，似然场地图生成可描述为算法1所示:

```
for all k do
	if p(z|uv) != z_max && p(z|uv) != z_min
		l = log(p(m|z,uv)/(1-p(m|z,uv)))
		p` = 1-(1\(1+exp(l)))
```

上述算法中，第三行表示计算占用概率，使用对数差异比的形式避免0或1附近具有较大数值的误差的概率值，第四行表示恢复概率值。对于多帧点云，重复上述算法，最终可获取概率栅格地图和占用栅格地图。如<u>图1</u>所示，为占用栅格地图可视化，其中(a)表示一次更新后的占用栅格地图，(b)表示多次更新后的占用栅格，(c)表示为多次更新后的占用栅格三值化形式。



针对似然场匹配，使用$\pi(p_i^{w})$表示似然场函数，最优化问题可表示为：
$$
x^{*}=\mathop{\arg\min}\limits_{k}\sum_{i=1}^{k}\lVert\pi(p_i^{W})\rVert \tag{式2}
$$
将激光雷达点云转换为地图坐标系，可简写表示如下：
$$
p_{i}'=SE2(T)*p_{i}
=\begin{vmatrix}
\cos{\theta}&-\sin{\theta}&x \\ \sin{\theta}&\cos{\theta}&y \\
0&0&1
\end{vmatrix} *
\begin{vmatrix} p_{x}\\ p_{y} \\ 1\end{vmatrix} \tag{式3}
$$
似然场往往使用图像存储，将点转换为图像坐标存在以下关系，其中$\alpha$表示为分辨率，$c$表示为偏移量：
$$
p_{i}^{f}=\alpha*p_{i}^{W}+c \tag{式4}
$$
根据高斯牛顿优化算法，对误差方程求取雅可比：
$$
\frac{\partial\pi}{\partial{x}}
=\frac{\partial\pi}{\partial{p_{i}^{W}}}
\frac{\partial{p_{i}^{W}}}{\partial{x}} \tag{式5}
$$
其中
$$
\frac{\partial\pi}{\partial{p_{i}^{W}}}
=\frac{\partial\pi}{\partial{p_{i}^{f}}}
\frac{\partial{p_{i}^{f}}}{\partial{p_{i}^{W}}}
=\alpha[\delta\pi_{x},\delta\pi_{y}] \tag{式6}
$$

$$
\frac{\partial{p_{i}^{W}}}{\partial{x}}
=\begin{vmatrix}1&0\\0&1\\-p_{x}\sin\theta-p_{y}\cos\theta&p_{x}\cos\theta+p_{y}\sin\theta \end{vmatrix}^{T} \tag{式7}
$$

​	

​	从似然场地图生成原理和可视化中能够得出结论，在一次或多次的的地图刷新中，占据和非占据往往非常接近，导致梯度呈现阶梯式变化，当初始位姿误差较大时，激光点索引周围像素大多数为非占据点或未知点，即存在梯度丢失现象，导致算法无法收敛，如<u>图2</u>所示现象。



对于二维图像$I(x,y)$，高斯平滑的二维形式为：


$$
G(x,y)=\frac{1}{2\pi{\sigma}^2}\int_{-\infty}^{+\infty}\int_{-\infty}^{+\infty}I(t_x,t_y)exp({-\frac{(x-t_x)^2+(y-t_y)^2}{2\sigma^2}})d(t_x)d(t_y) \tag{式8}
$$
针对梯度存在较大变化的图像运用高斯平滑可使梯度平滑，如图3所示，标准差为4像素：

![Figure_1](./Figure_1.png)

​													图4 图像运用高斯平滑梯度可视化

​	对似然场运用高斯平滑会导致边界处不够分明影响匹配精度，本文将高斯平滑后的似然场和原始似然场融合构成配准模块，最优化模块可表示为：
$$
x^{*}=\mathop{\arg\min}\limits_{k}\sum_{i=1}^{k}\lVert\pi(p_i^{W})+\pi_{gauss}{(p_i^W)}\rVert \tag{式9}
$$


# 4 实验与结果分析

​	实验采用的硬件为Intel Core I5-1340P CPU，内存为16G，软件环境为Ubuntu 22.04.4 LTS操作系统，实验数据采用***数据集和自采数据集，自采数据使用ros2 humble记录。



# 5 结论



# 参考文献：

[1] 李枭凯,李广云,索世恒,等.激光SLAM技术进展[J].导航定位学报,2023,11(04):8-17.DOI:10.16547/j.cnki.10-1096.20230402.

[2] Hess W, Kohler D, Rapp H, et al. Real-time loop closure in 2D LIDAR SLAM[C]//2016 IEEE international conference on robotics and automation (ICRA). IEEE, 2016: 1271-1278.

[3] Konolige K, Grisetti G, Kümmerle R, et al. Efficient sparse pose  adjustment for 2D mapping[C]//2010 IEEE/RSJ International Conference on  Intelligent Robots and Systems. IEEE, 2010: 22-29.

[4]Grisetti G, Stachniss C, Burgard W.  Improved techniques for grid mapping with rao-blackwellized particle  filters[J]. IEEE transactions on Robotics, 2007, 23(1): 34-46.

[5] 张亮,刘智宇,曹晶瑛,等.扫地机器人增强位姿融合的Cartographer算法及系统实现[J].软件学报,2020,31(09):2678-2690.DOI:10.13328/j.cnki.jos.005937.

[6] Olson E B. Real-time correlative scan matching[C]//2009 IEEE International Conference on Robotics and Automation. IEEE, 2009: 4387-4393.

[7] 郝宇,张亿,黄磊,等.基于改进图优化的移动机器人二维激光SLAM算法研究[J/OL].激光与光电子学进展,1-13[2024-11-01].http://kns.cnki.net/kcms/detail/31.1690.TN.20240517.1421.004.html.

[8] 汪方斌,曹锟,龚雪,等.基于因子图优化的光伏电站场景激光雷达SLAM方法[J].激光与光电子学进展,2024,61(14):165-173.

[9] Wen J, Zhang X, Gao H, et al.  CAE-RLSM: Consistent and efficient redundant line segment merging for  online feature map building[J]. IEEE Transactions on Instrumentation and Measurement, 2019, 69(7): 4222-4237.

[10] Han J, Hu Z, Yang S, et al. SoMaSLAM: 2D Graph SLAM for Sparse Range  Sensing with Soft Manhattan World Constraints[J]. arXiv preprint  arXiv:2409.15736, 2024.

[11] 郝宇,张亿,黄磊,等.基于改进图优化的移动机器人二维激光SLAM算法研究[J/OL].激光与光电子学进展,1-13[2024-11-12].http://kns.cnki.net/kcms/detail/31.1690.TN.20240517.1421.004.html.

[12] Xu W, Zhang F. Fast-lio: A fast,  robust lidar-inertial odometry package by tightly-coupled iterated  kalman filter[J]. IEEE Robotics and Automation Letters, 2021, 6(2):  3317-3324.

[13] Bai C, Xiao T, Chen Y, et al.  Faster-LIO: Lightweight tightly coupled LiDAR-inertial odometry using  parallel sparse incremental voxels[J]. IEEE Robotics and Automation  Letters, 2022, 7(2): 4861-4868.