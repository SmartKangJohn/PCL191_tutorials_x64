# PCL191_tutorials_x64
为了学习PCL，将PCL1.9.1的tutorials建为工程并解决其中的BUG问题 

注： 项目更新中...

## 安装包要求及环境：
1、win10+vs2017+pcl1.9.1_x64;

2、hdf5-1.12.0;

### project 为工程项目
### sources 为工程源码

## 操作说明
使用cmake3.15及以上版本，选择该项目的sources生成到project中即可。
camke会自动链接pcl的相关路径，只有hadf5的路径需要自己配一下。

第一次发开源，文档排版可能乱请见谅！
有问题请留issue，闲暇之余可能回复，哈哈哈哈！


# 工程介绍区
1.build_tree bug未解决

2.cloud_view
介绍了cloud_viewer的可视化操作；

3.cluster_extraction
介绍了聚类分割的操作；
代码中无可视化pcd， 故写了个函数在 一个window中分出多个viewer视窗来分别显示.pcd文件；

4.concatenate_clouds
介绍了点云的 点连接(+) 与 字段连接 的区别(concatenateFields)；

5.concatenate_fields
介绍了点云的不同域（字段）连接，如点+法向量；

6.concatenate_points
介绍了点云的点连接，扩充点云数量(width)；

7.concave_hull_2d
介绍提取点云2d凹面——sac滤波，投影滤波，提取2d凹面；

8.conditional_removal
介绍移除离群点——条件移除滤波器；

9.convex_hull_2d
介绍提取点云2d凸包——sac滤波，投影滤波，提取2d凸包；

10.correspondence_grouping

11.cylinder_segmentation

12.

13.

14.

15.

16.

17.

18.

19.

20.